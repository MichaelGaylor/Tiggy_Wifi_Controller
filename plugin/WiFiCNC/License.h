/*
 * Tiggy Motion Controller - License Validation
 *
 * Simple offline license key check. No server, no internet, no DRM.
 * Free tier: 3 axes (X/Y/Z).  Pro tier: 6 axes + I/O module + threading.
 */

#ifndef LICENSE_H
#define LICENSE_H

#include <stdbool.h>

/* Check if a license key is valid for the given email.
 * Returns true if the key unlocks Pro features. */
bool License_CheckKey(const char *email, const char *key);

/* Generate a display string for the current license tier. */
const char *License_TierName(bool isPro);

#endif /* LICENSE_H */
