# mDNS Discovery Component

Query mDNS services on the local network to discover peers.

## Features

- Periodic mDNS queries for specified service type
- Automatic peer timeout and cleanup
- Callbacks for peer found/lost events
- Platform sensors for peer count and list

## Configuration

```yaml
# Announce yourself (standard ESPHome mDNS)
mdns:
  services:
    - service: "_udp-intercom"
      protocol: "_udp"
      port: 12346

# Discover others
mdns_discovery:
  id: peer_discovery
  service_type: "udp-intercom"  # Without underscore prefix
  scan_interval: 10s
  peer_timeout: 60s
  on_peer_found:
    - logger.log:
        format: "Found: %s at %s:%d"
        args: [name.c_str(), ip.c_str(), port]
  on_peer_lost:
    - logger.log:
        format: "Lost: %s"
        args: [name.c_str()]
  on_scan_complete:
    - logger.log:
        format: "Scan complete, %d peers"
        args: [peer_count]
```

## Actions

```yaml
# Manual scan trigger
- mdns_discovery.scan:
    id: peer_discovery
```

## Sensors

```yaml
sensor:
  - platform: mdns_discovery
    mdns_discovery_id: peer_discovery
    peer_count:
      name: "Peer Count"

text_sensor:
  - platform: mdns_discovery
    mdns_discovery_id: peer_discovery
    peers_list:
      name: "Discovered Peers"
```

## Lambda Access

```yaml
- lambda: |-
    int count = id(peer_discovery).get_peer_count();
    if (count > 0) {
      std::string ip = id(peer_discovery).get_peer_ip(0);
      std::string name = id(peer_discovery).get_peer_name(0);
      ESP_LOGI("p2p", "First peer: %s (%s)", name.c_str(), ip.c_str());
    }

    // Get IP by name
    std::string ip = id(peer_discovery).get_peer_ip_by_name("intercom-2");

    // Get formatted list
    std::string list = id(peer_discovery).get_peers_list();
```
