/*
 * Aircraft registration to ICAO hex lookup database
 *
 * Loads tar1090-db aircraft.csv and provides fast binary-search lookup
 * of registration -> ICAO hex address.
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <ctype.h>
#include <err.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "aircraft_db.h"

#define DB_URL "https://github.com/wiedehopf/tar1090-db/raw/refs/heads/csv/aircraft.csv.gz"

typedef struct {
    char reg[12];   /* registration (e.g., "N712TW") */
    char hex[7];    /* ICAO hex (e.g., "A98539") */
} db_entry_t;

static db_entry_t *db = NULL;
static int db_count = 0;
static int db_capacity = 0;

static int entry_cmp(const void *a, const void *b)
{
    return strcmp(((const db_entry_t *)a)->reg,
                 ((const db_entry_t *)b)->reg);
}

/*
 * Normalize a registration string: strip leading dots, uppercase,
 * remove dashes/spaces.
 */
static void normalize_reg(const char *src, char *dst, int dstlen)
{
    /* Skip leading dots */
    while (*src == '.') src++;

    int j = 0;
    for (int i = 0; src[i] && j < dstlen - 1; i++) {
        if (src[i] == '-' || src[i] == ' ')
            continue;
        dst[j++] = toupper((unsigned char)src[i]);
    }
    dst[j] = '\0';
}

int aircraft_db_load(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "aircraft_db: cannot open %s\n", path);
        return -1;
    }

    aircraft_db_destroy();

    db_capacity = 700000;
    db = malloc(sizeof(db_entry_t) * db_capacity);
    if (!db) {
        fclose(f);
        return -1;
    }

    char line[512];
    int loaded = 0;
    while (fgets(line, sizeof(line), f)) {
        /* Format: icao_hex;registration;type;flags;description;... */
        char *hex_field = line;
        char *semi1 = strchr(line, ';');
        if (!semi1) continue;
        *semi1 = '\0';
        char *reg_field = semi1 + 1;
        char *semi2 = strchr(reg_field, ';');
        if (semi2) *semi2 = '\0';

        /* Validate hex (6 chars, hex digits) */
        int hex_len = strlen(hex_field);
        if (hex_len != 6) continue;
        int hex_ok = 1;
        for (int i = 0; i < 6; i++) {
            char c = hex_field[i];
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
                  (c >= 'a' && c <= 'f'))) {
                hex_ok = 0;
                break;
            }
        }
        if (!hex_ok) continue;

        /* Skip empty registrations */
        if (!reg_field[0]) continue;

        if (db_count >= db_capacity) {
            db_capacity *= 2;
            db_entry_t *new_db = realloc(db, sizeof(db_entry_t) * db_capacity);
            if (!new_db) break;
            db = new_db;
        }

        normalize_reg(reg_field, db[db_count].reg, sizeof(db[db_count].reg));
        if (!db[db_count].reg[0]) continue;

        /* Store hex uppercase */
        for (int i = 0; i < 6; i++)
            db[db_count].hex[i] = toupper((unsigned char)hex_field[i]);
        db[db_count].hex[6] = '\0';

        db_count++;
        loaded++;
    }

    fclose(f);

    /* Sort by registration for binary search */
    qsort(db, db_count, sizeof(db_entry_t), entry_cmp);

    fprintf(stderr, "aircraft_db: loaded %d entries from %s\n", loaded, path);
    return loaded;
}

const char *aircraft_db_lookup(const char *registration)
{
    if (!db || db_count == 0 || !registration || !registration[0])
        return NULL;

    char norm[12];
    normalize_reg(registration, norm, sizeof(norm));
    if (!norm[0]) return NULL;

    db_entry_t key;
    strncpy(key.reg, norm, sizeof(key.reg));
    key.reg[sizeof(key.reg) - 1] = '\0';

    db_entry_t *found = bsearch(&key, db, db_count, sizeof(db_entry_t),
                                 entry_cmp);
    return found ? found->hex : NULL;
}

void aircraft_db_destroy(void)
{
    free(db);
    db = NULL;
    db_count = 0;
    db_capacity = 0;
}

const char *aircraft_db_default_path(void)
{
    static char path[512];
    const char *home = getenv("HOME");
    if (!home) return NULL;
    snprintf(path, sizeof(path), "%s/.iridium-sniffer/aircraft.csv", home);
    return path;
}

int aircraft_db_update(void)
{
    const char *path = aircraft_db_default_path();
    if (!path) {
        warnx("aircraft_db: cannot determine HOME directory");
        return -1;
    }

    /* Create directory */
    char dir[512];
    snprintf(dir, sizeof(dir), "%s/.iridium-sniffer", getenv("HOME"));
    mkdir(dir, 0755);

    /* Download and decompress */
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
             "curl -sL '%s' | gunzip > '%s.tmp' && mv '%s.tmp' '%s'",
             DB_URL, path, path, path);

    fprintf(stderr, "aircraft_db: downloading from tar1090-db...\n");
    int ret = system(cmd);
    if (ret != 0) {
        warnx("aircraft_db: download failed (is curl installed?)");
        return -1;
    }

    struct stat st;
    if (stat(path, &st) != 0 || st.st_size < 1000) {
        warnx("aircraft_db: downloaded file is too small or missing");
        unlink(path);
        return -1;
    }

    fprintf(stderr, "aircraft_db: saved to %s (%.1f MB)\n",
            path, st.st_size / 1e6);
    return 0;
}
