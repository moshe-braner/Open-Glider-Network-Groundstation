extern int config_done;

extern String ogn_ssid[];
extern String ogn_wpass[];

extern int ssid_index;

extern float   ogn_lat;
extern float   ogn_lon;
extern int     ogn_alt;
extern int16_t ogn_geoid_separation;

extern int largest_range;
extern uint32_t farthest_ID;
extern float farthest_lat;
extern float farthest_lon;
extern float farthest_alt;

extern bool    ogn_mobile;
extern uint8_t  ogn_band;
extern uint8_t  ogn_protocol_1;
extern uint8_t  ogn_protocol_2;
extern bool     ogn_bec;
extern uint8_t  noise_sampling;

extern String   ogn_callsign;
extern String   ogn_server;
extern uint16_t ogn_port;

extern bool     ogn_debug;
extern uint16_t ogn_debugport;
extern bool     ogn_itrackbit;
extern bool     ogn_istealthbit;
extern uint16_t ogn_range;
extern bool     ogn_hiderelayed;

extern int8_t   ogn_sleepmode;
extern int8_t   ogn_timezone;
extern int8_t   ogn_morning;
extern int8_t   ogn_evening;
extern uint16_t ogn_rxidle;
extern uint16_t ogn_wakeuptimer;

extern bool     fanet_enable;
extern bool     zabbix_enable;
extern String   zabbix_server;
extern uint16_t zabbix_port;
extern String zabbix_key;

extern bool beers_show;

extern bool      remotelogs_enable;
extern String    remotelogs_server;
extern uint16_t  remotelogs_port;

extern unsigned long   oled_disable;

extern bool  testmode_enable;
extern bool  private_network;
extern unsigned int  webserver_port;
extern bool  new_protocol_enable;
extern String new_protocol_server;
extern uint32_t new_protocol_port;

extern bool ognrelay_enable;
extern bool ognrelay_base;
extern bool ognrelay_time;
extern bool ognreverse_time;
extern bool ogn_gnsstime;
extern uint32_t ognrelay_key;
extern bool ognvoltage_limit;

extern bool time_master;
extern bool time_client;
