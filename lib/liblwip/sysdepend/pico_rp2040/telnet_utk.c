/*
 *      telnet_utk.c
 *      See telnet_utk.h.  lwIP raw TCP API, runs in cyw43_utk_task only.
 *
 *      The console text comes from a second ring that usb_console.c fills
 *      next to the USB ring while a client is connected.  Line ends become
 *      CR LF (Telnet).  Anything the client sends (Telnet option
 *      negotiation, keys) is read and thrown away.
 */

#include "lwip/tcp.h"
#include "telnet_utk.h"
#include <tm/tmonitor.h>

/* usb_console.c (kernel side), plain types */
extern void tm_net_console_enable(int on);
extern int  tm_net_console_get(unsigned char * out, int max);

#define CHUNK 256 /* console bytes per write (<= 2x after CR LF) */

static struct tcp_pcb * listener;
static struct tcp_pcb * client;

static void drop_client (void)
{
    if (client != NULL)
    {
        tcp_arg(client, NULL);
        tcp_recv(client, NULL);
        tcp_err(client, NULL);
        if (tcp_close(client) != ERR_OK)
        {
            tcp_abort(client);
        }
        client = NULL;
    }
    tm_net_console_enable(0);
}

static void client_err (void * arg, err_t err)
{
    (void) arg;
    (void) err;
    client = NULL; /* lwIP has already freed the pcb */
    tm_net_console_enable(0);
}

static err_t client_recv (void * arg, struct tcp_pcb * pcb, struct pbuf * p,
                          err_t err)
{
    (void) arg;
    (void) err;
    if (p == NULL)
    { /* the client closed */
        if (pcb == client)
        {
            drop_client();
        }
        return ERR_OK;
    }
    tcp_recved(pcb, p->tot_len); /* input ignored */
    pbuf_free(p);
    return ERR_OK;
}

static err_t on_accept (void * arg, struct tcp_pcb * pcb, err_t err)
{
    static const char hello[] =
        "\r\n=== robot car console over Wi-Fi (output only) ===\r\n";

    (void) arg;
    if (err != ERR_OK || pcb == NULL)
    {
        return ERR_VAL;
    }
    if (client != NULL)
    {
        drop_client(); /* newest client wins */
    }
    client = pcb;
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, client_recv);
    tcp_err(pcb, client_err);
    tcp_nagle_disable(pcb);
    (void) tcp_write(pcb, hello, sizeof(hello) - 1, TCP_WRITE_FLAG_COPY);
    (void) tcp_output(pcb);
    tm_net_console_enable(1);
    return ERR_OK;
}

void telnet_utk_start (void)
{
    struct tcp_pcb * pcb;

    if (listener != NULL)
    {
        return;
    }
    pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (pcb == NULL)
    {
        return;
    }
    if (tcp_bind(pcb, IP_ANY_TYPE, TELNET_UTK_PORT) != ERR_OK)
    {
        tcp_close(pcb);
        return;
    }
    listener = tcp_listen(pcb);
    if (listener == NULL)
    {
        return;
    }
    tcp_accept(listener, on_accept);
    tm_printf((UB *) "telnet: console on port %d (Tera Term: TCP/IP, Telnet)\n",
              TELNET_UTK_PORT);
}

void telnet_utk_poll (void)
{
    unsigned char in[CHUNK];
    char          out[2 * CHUNK];
    int           n, i, k;

    if (client == NULL)
    {
        return;
    }
    /* room for the worst case: every byte a line end */
    n = (int) tcp_sndbuf(client) / 2;
    if (n <= 0)
    {
        return;
    }
    if (n > CHUNK)
    {
        n = CHUNK;
    }
    n = tm_net_console_get(in, n);
    if (n <= 0)
    {
        return;
    }
    for (i = 0, k = 0; i < n; i++)
    {
        if (in[i] == '\n')
        {
            out[k++] = '\r';
        }
        out[k++] = (char) in[i];
    }
    if (tcp_write(client, out, (u16_t) k, TCP_WRITE_FLAG_COPY) == ERR_OK)
    {
        (void) tcp_output(client);
    }
    /* on failure the text is lost: the console never waits */
}

/*** end of file ***/
