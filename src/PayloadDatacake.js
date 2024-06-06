function Decoder(input) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};
  if (input.length==1 && input[0]==0xFF) {
      decoded.fix = false;
  } else {
      decoded.fix=true;
      decoded.latitude = ((input[0]<<16)>>>0) + ((input[1]<<8)>>>0) + input[2];
      decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;
    
      decoded.longitude = ((input[3]<<16)>>>0) + ((input[4]<<8)>>>0) + input[5];
      decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;
      decoded.location = "(" + decoded.latitude + "," + decoded.longitude + ")";
      var altValue = ((input[6]<<8)>>>0) + input[7];
      var sign = input[6] & (1 << 7);
      if(sign)
      {
        decoded.altitude = 0xFFFF0000 | altValue;
      }
      else
      {
        decoded.altitude = altValue;
      }
    
      decoded.hdop = input[8] / 10.0;
  }
	try {
        decoded.LORA_RSSI = (!!normalizedPayload.gateways && !!normalizedPayload.gateways[0] && normalizedPayload.gateways[0].rssi) || 0;
        decoded.LORA_SNR = (!!normalizedPayload.gateways && !!normalizedPayload.gateways[0] && normalizedPayload.gateways[0].snr) || 0;
        decoded.LORA_DATARATE = normalizedPayload.data_rate;   	    
	} catch (e) {
	    console.log(e);
	}

  return decoded;
}
