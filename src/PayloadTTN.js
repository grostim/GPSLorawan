/**
 *
 * To decode the binary payload, you can use the following
 * javascript decoder function. It should work with the TTN console (V3).
 *
 **/
function decodeUplink(input) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  if (input.bytes[0]== 0xFF && input.bytes.length== 1) {
    // If the first and only byte is 0xFF, then we have no GPS fix 
    decoded.fix = false;
    return {
      data:decoded,
    }
  }
  else
  {
    decoded.fix = true;
    decoded.latitude = ((input.bytes[0]<<16)>>>0) + ((input.bytes[1]<<8)>>>0) + input.bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;
  
    decoded.longitude = ((input.bytes[3]<<16)>>>0) + ((input.bytes[4]<<8)>>>0) + input.bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;
  
    var altValue = ((input.bytes[6]<<8)>>>0) + input.bytes[7];
    var sign = input.bytes[6] & (1 << 7);
    if(sign)
    {
      decoded.altitude = 0xFFFF0000 | altValue;
    }
    else
    {
      decoded.altitude = altValue;
    }
  
    decoded.hdop = input.bytes[8] / 10.0;
  
    return {
      data:decoded,
    }
  }
}

function encodeDownlink(input) {
/**
 * Exemple of JSON Payload:
  {
    "Cmd": "ConfigDistanceInterval",
    "value": 300
  }
**/
  var ret = [];
  var value;
    if (input.data.Cmd == "ConfigTimeInterval")
    {
        value = input.data.value;
        
        ret[0] = 0x01
        ret[1] = value >>8;
        ret[2] = (value & 0xFF)
    }
    else if (input.data.Cmd == "ConfigDistanceInterval")
    {
        value = input.data.value;
        
        ret[0] = 0x02
        ret[1] = value >>8;
        ret[2] = (value & 0xFF);
    }  
    
  
  return {
    bytes: ret,
    fPort: 1,
    warnings: [],
    errors: []
  };
}

