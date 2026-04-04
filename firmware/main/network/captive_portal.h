/*
 * Captive Portal - WiFi Setup via Web Browser
 *
 * When running in AP mode, starts a simple HTTP server and DNS
 * redirect so users can configure WiFi credentials from any
 * browser without needing the Protocol Tester.
 */

#ifndef CAPTIVE_PORTAL_H
#define CAPTIVE_PORTAL_H

/* Start the captive portal (call after AP mode is active) */
void captive_portal_start(void);

/* Stop the captive portal (call before switching to STA mode) */
void captive_portal_stop(void);

#endif /* CAPTIVE_PORTAL_H */
