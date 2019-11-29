#ifndef NODE_CONFIG_H__
#define NODE_CONFIG_H__

void set_node_config(uint32_t config);

uint32_t get_node_config(void);

void clear_node_config(void);

void node_config_flash_init(void);

#endif  /* NODE_CONFIG_H__ */