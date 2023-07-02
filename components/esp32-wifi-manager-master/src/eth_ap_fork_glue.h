void *eth_fork_netif_glue(esp_eth_handle_t eth_hdl);

void wifi_init_softap(void);
err_t eth_copyable_init(struct netif *netif);
void eth_copyable_input(void *h, void *buffer, size_t len, void *eb);

struct esp_netif_lwip_vanilla_config {
    err_t (*init_fn)(struct netif*);
    void (*input_fn)(void *netif, void *buffer, size_t len, void *eb);
};