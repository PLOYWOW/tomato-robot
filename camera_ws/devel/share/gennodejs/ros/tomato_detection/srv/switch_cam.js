// Auto-generated. Do not edit!

// (in-package tomato_detection.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class switch_camRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.req_status = null;
    }
    else {
      if (initObj.hasOwnProperty('req_status')) {
        this.req_status = initObj.req_status
      }
      else {
        this.req_status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type switch_camRequest
    // Serialize message field [req_status]
    bufferOffset = _serializer.bool(obj.req_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type switch_camRequest
    let len;
    let data = new switch_camRequest(null);
    // Deserialize message field [req_status]
    data.req_status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tomato_detection/switch_camRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87a8ff803de57df7fedeb6b9a9d38f8d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool req_status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new switch_camRequest(null);
    if (msg.req_status !== undefined) {
      resolved.req_status = msg.req_status;
    }
    else {
      resolved.req_status = false
    }

    return resolved;
    }
};

class switch_camResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type switch_camResponse
    // Serialize message field [data]
    bufferOffset = _serializer.uint8(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type switch_camResponse
    let len;
    let data = new switch_camResponse(null);
    // Deserialize message field [data]
    data.data = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tomato_detection/switch_camResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c8164229e7d2c17eb95e9231617fdee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new switch_camResponse(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: switch_camRequest,
  Response: switch_camResponse,
  md5sum() { return 'c1d0f6b5d3ccfee714fe1fdff0074c75'; },
  datatype() { return 'tomato_detection/switch_cam'; }
};
