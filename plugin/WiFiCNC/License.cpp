/*
 * Tiggy Motion Controller - License Validation
 *
 * Offline SHA-256 key check. The salt is injected at build time via
 * /DTIGGY_LICENSE_SALT="..." in the build script.  Without the salt,
 * the build produces a free-only DLL (no way to unlock Pro).
 */

#include "License.h"
#include <windows.h>
#include <wincrypt.h>
#include <cstring>
#include <cstdio>
#include <cctype>

#ifndef TIGGY_LICENSE_SALT
#define TIGGY_LICENSE_SALT ""   /* No salt = free-only build */
#endif

/* Simple SHA-256 using Windows CryptoAPI (no external dependencies) */
static bool SHA256Hex(const char *input, size_t len, char hexOut[65])
{
    HCRYPTPROV hProv = 0;
    HCRYPTHASH hHash = 0;
    BYTE hash[32];
    DWORD hashLen = 32;

    if (!CryptAcquireContextA(&hProv, nullptr, nullptr, PROV_RSA_AES,
                               CRYPT_VERIFYCONTEXT))
        return false;

    if (!CryptCreateHash(hProv, CALG_SHA_256, 0, 0, &hHash)) {
        CryptReleaseContext(hProv, 0);
        return false;
    }

    CryptHashData(hHash, (const BYTE *)input, (DWORD)len, 0);
    CryptGetHashParam(hHash, HP_HASHVAL, hash, &hashLen, 0);

    for (int i = 0; i < 32; i++)
        sprintf_s(hexOut + i * 2, 3, "%02x", hash[i]);
    hexOut[64] = '\0';

    CryptDestroyHash(hHash);
    CryptReleaseContext(hProv, 0);
    return true;
}

/* Lowercase and trim whitespace from email for consistent hashing */
static void NormalizeEmail(const char *email, char *out, size_t outSize)
{
    size_t j = 0;
    /* Skip leading whitespace */
    while (*email && isspace((unsigned char)*email)) email++;
    for (size_t i = 0; email[i] && j < outSize - 1; i++) {
        if (!isspace((unsigned char)email[i]) || email[i] == ' ')
            out[j++] = (char)tolower((unsigned char)email[i]);
    }
    /* Trim trailing whitespace */
    while (j > 0 && isspace((unsigned char)out[j - 1])) j--;
    out[j] = '\0';
}

bool License_CheckKey(const char *email, const char *key)
{
    if (!email || !key || email[0] == '\0' || key[0] == '\0')
        return false;

    /* No salt = free-only build, no valid keys possible */
    const char *salt = TIGGY_LICENSE_SALT;
    if (salt[0] == '\0')
        return false;

    /* Normalize email */
    char normEmail[256];
    NormalizeEmail(email, normEmail, sizeof(normEmail));

    /* Build hash input: email + salt */
    char hashInput[512];
    snprintf(hashInput, sizeof(hashInput), "%s%s", normEmail, salt);

    /* Compute SHA-256 */
    char expected[65];
    if (!SHA256Hex(hashInput, strlen(hashInput), expected))
        return false;

    /* Compare (case-insensitive, first 32 hex chars = 16 bytes) */
    char normKey[65] = {0};
    for (int i = 0; i < 64 && key[i]; i++)
        normKey[i] = (char)tolower((unsigned char)key[i]);

    return strncmp(expected, normKey, 64) == 0;
}

const char *License_TierName(bool isPro)
{
    return isPro ? "Tiggy Pro (6-axis)" : "Tiggy Free (3-axis)";
}
