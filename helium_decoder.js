function Decoder(bytes, port, uplink_info) {
    var decoded = {};
    decoded.temperature = (bytes[1] << 8 | (bytes[2])) / 100;
    decoded.humidity = (bytes[3] << 8 | (bytes[4]))/100;
    decoded.pressure = (bytes[8] | (bytes[7] << 8) | (bytes[6] << 16) | (bytes[5] << 24))/100;
    decoded.gas = ( bytes[12] | (bytes[11] << 8) | (bytes[10] << 16) | (bytes[9] << 24))/1000;
    decoded.battery = (bytes[13]);
    decoded.light = (bytes[14] << 8 | (bytes[15]))/100;
/* someday ..

  The uplink_info variable is an OPTIONAL third parameter that provides the following:

  uplink_info = {
    type: "join",
    uuid: <UUIDv4>,
    id: <device id>,
    name: <device name>,
    dev_eui: <dev_eui>,
    app_eui: <app_eui>,
    metadata: {...},
    fcnt: <integer>,
    reported_at: <timestamp>,
    port: <integer>,
    devaddr: <devaddr>,
    hotspots: {...},
    hold_time: <integer>
  }
*/

  if (uplink_info) {
    // do something with uplink_info fields
  }

  return decoded;
}
