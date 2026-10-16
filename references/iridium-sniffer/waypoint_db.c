/*
 * Aviation waypoint/fix database lookup
 *
 * Loads a CSV of 5-letter ICAO waypoint identifiers with lat/lon and
 * provides fast binary-search lookup. Used as a last-resort position
 * source when ACARS messages reference waypoints by name without
 * explicit coordinates.
 *
 * Database sources: X-Plane earth_fix.dat (GPL v3),
 * Global Aviation Waypoints (MIT).
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "waypoint_db.h"

typedef struct {
    char ident[6];  /* 5-letter ICAO fix identifier + null */
    double lat;
    double lon;
} wp_entry_t;

static wp_entry_t *wpdb = NULL;
static int wpdb_count = 0;

static int wp_cmp(const void *a, const void *b)
{
    return strcmp(((const wp_entry_t *)a)->ident,
                 ((const wp_entry_t *)b)->ident);
}

int waypoint_db_load(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "waypoint_db: cannot open %s\n", path);
        return -1;
    }

    waypoint_db_destroy();

    int capacity = 130000;
    wpdb = malloc(sizeof(wp_entry_t) * capacity);
    if (!wpdb) {
        fclose(f);
        return -1;
    }

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        /* Format: IDENT,LAT,LON */
        char *id = line;
        char *c1 = strchr(line, ',');
        if (!c1) continue;
        *c1 = '\0';
        char *lat_s = c1 + 1;
        char *c2 = strchr(lat_s, ',');
        if (!c2) continue;
        *c2 = '\0';
        char *lon_s = c2 + 1;

        /* Strip trailing whitespace from lon */
        int llen = strlen(lon_s);
        while (llen > 0 && (lon_s[llen-1] == '\n' || lon_s[llen-1] == '\r'))
            lon_s[--llen] = '\0';

        /* Validate: must be 2-5 letter uppercase identifier */
        int idlen = strlen(id);
        if (idlen < 2 || idlen > 5) continue;

        if (wpdb_count >= capacity) {
            capacity *= 2;
            wp_entry_t *new_db = realloc(wpdb, sizeof(wp_entry_t) * capacity);
            if (!new_db) break;
            wpdb = new_db;
        }

        strncpy(wpdb[wpdb_count].ident, id, 5);
        wpdb[wpdb_count].ident[5] = '\0';
        wpdb[wpdb_count].lat = atof(lat_s);
        wpdb[wpdb_count].lon = atof(lon_s);
        wpdb_count++;
    }

    fclose(f);

    qsort(wpdb, wpdb_count, sizeof(wp_entry_t), wp_cmp);

    fprintf(stderr, "waypoint_db: loaded %d fixes from %s\n", wpdb_count, path);
    return wpdb_count;
}

int waypoint_db_lookup(const char *ident, double *lat, double *lon)
{
    if (!wpdb || wpdb_count == 0 || !ident || !ident[0])
        return 0;

    wp_entry_t key;
    strncpy(key.ident, ident, 5);
    key.ident[5] = '\0';

    wp_entry_t *found = bsearch(&key, wpdb, wpdb_count, sizeof(wp_entry_t),
                                 wp_cmp);
    if (!found) return 0;

    *lat = found->lat;
    *lon = found->lon;
    return 1;
}

void waypoint_db_destroy(void)
{
    free(wpdb);
    wpdb = NULL;
    wpdb_count = 0;
}
