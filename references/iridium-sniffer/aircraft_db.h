/*
 * Aircraft registration to ICAO hex lookup database
 *
 * Loads the tar1090-db aircraft.csv format (semicolon-separated,
 * fields: icao_hex;registration;type;flags;description;...)
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef __AIRCRAFT_DB_H__
#define __AIRCRAFT_DB_H__

/*
 * Load aircraft database from CSV file.
 * Expected format: icao_hex;registration;... (semicolon-separated)
 * Returns number of entries loaded, or -1 on error.
 */
int aircraft_db_load(const char *path);

/*
 * Look up ICAO hex address by aircraft registration.
 * Returns pointer to 6-char hex string (static, valid until next load),
 * or NULL if not found.
 */
const char *aircraft_db_lookup(const char *registration);

/*
 * Free all database resources.
 */
void aircraft_db_destroy(void);

/*
 * Download/update the aircraft database from the tar1090-db project.
 * Stores at ~/.iridium-sniffer/aircraft.csv
 * Returns 0 on success, -1 on error.
 */
int aircraft_db_update(void);

/*
 * Get default database path (~/.iridium-sniffer/aircraft.csv).
 * Returns static buffer.
 */
const char *aircraft_db_default_path(void);

#endif
