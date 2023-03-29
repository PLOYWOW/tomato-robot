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

class SelectTomatoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gripperPos = null;
    }
    else {
      if (initObj.hasOwnProperty('gripperPos')) {
        this.gripperPos = initObj.gripperPos
      }
      else {
        this.gripperPos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SelectTomatoRequest
    // Serialize message field [gripperPos]
    bufferOffset = _arraySerializer.float32(obj.gripperPos, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SelectTomatoRequest
    let len;
    let data = new SelectTomatoRequest(null);
    // Deserialize message field [gripperPos]
    data.gripperPos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.gripperPos.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tomato_detection/SelectTomatoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b4a1966605e01c4d22aa4278d4a6590';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] gripperPos
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SelectTomatoRequest(null);
    if (msg.gripperPos !== undefined) {
      resolved.gripperPos = msg.gripperPos;
    }
    else {
      resolved.gripperPos = []
    }

    return resolved;
    }
};

class SelectTomatoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tomatoPos = null;
    }
    else {
      if (initObj.hasOwnProperty('tomatoPos')) {
        this.tomatoPos = initObj.tomatoPos
      }
      else {
        this.tomatoPos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SelectTomatoResponse
    // Serialize message field [tomatoPos]
    bufferOffset = _arraySerializer.float32(obj.tomatoPos, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SelectTomatoResponse
    let len;
    let data = new SelectTomatoResponse(null);
    // Deserialize message field [tomatoPos]
    data.tomatoPos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.tomatoPos.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tomato_detection/SelectTomatoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ec060072f29f1fa2698deb242ab0c3e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] tomatoPos
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SelectTomatoResponse(null);
    if (msg.tomatoPos !== undefined) {
      resolved.tomatoPos = msg.tomatoPos;
    }
    else {
      resolved.tomatoPos = []
    }

    return resolved;
    }
};

module.exports = {
  Request: SelectTomatoRequest,
  Response: SelectTomatoResponse,
  md5sum() { return '740697c9bd3592ba5c586821d8c52d19'; },
  datatype() { return 'tomato_detection/SelectTomato'; }
};
