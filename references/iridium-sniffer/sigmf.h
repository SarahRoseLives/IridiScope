/*
 * SigMF metadata reader for auto-configuring from .sigmf-meta files
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef __SIGMF_H__
#define __SIGMF_H__

/*
 * Parse a .sigmf-meta JSON file and extract sample rate, center frequency,
 * and data format. Returns 0 on success, -1 on failure.
 *
 * Output parameters are only written when the corresponding field is present
 * in the metadata. Callers should initialize them to sentinel values before
 * calling to detect which fields were populated.
 */
int sigmf_read_meta(const char *meta_path,
                    double *sample_rate,
                    double *center_freq,
                    int *iq_format);

#endif
