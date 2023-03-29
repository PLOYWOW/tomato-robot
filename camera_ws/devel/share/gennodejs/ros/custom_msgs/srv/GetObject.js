// Auto-generated. Do not edit!

// (in-package custom_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetObjectRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.item = null;
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('item')) {
        this.item = initObj.item
      }
      else {
        this.item = '';
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetObjectRequest
    // Serialize message field [item]
    bufferOffset = _serializer.string(obj.item, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetObjectRequest
    let len;
    let data = new GetObjectRequest(null);
    // Deserialize message field [item]
    data.item = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.item);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'custom_msgs/GetObjectRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '582eeac6e6e1af2d7d31aa5c5948e265';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string item
    int64 id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetObjectRequest(null);
    if (msg.item !== undefined) {
      resolved.item = msg.item;
    }
    else {
      resolved.item = ''
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

class GetObjectResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos_width = null;
    }
    else {
      if (initObj.hasOwnProperty('pos_width')) {
        this.pos_width = initObj.pos_width
      }
      else {
        this.pos_width = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetObjectResponse
    // Check that the constant length array field [pos_width] has the right length
    if (obj.pos_width.length !== 4) {
      throw new Error('Unable to serialize array field pos_width - length must be 4')
    }
    // Serialize message field [pos_width]
    bufferOffset = _arraySerializer.float64(obj.pos_width, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetObjectResponse
    let len;
    let data = new GetObjectResponse(null);
    // Deserialize message field [pos_width]
    data.pos_width = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'custom_msgs/GetObjectResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7d5ccf2629aed135906cf9b282336d7f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[4] pos_width #[x,y,z,grasp_width]
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetObjectResponse(null);
    if (msg.pos_width !== undefined) {
      resolved.pos_width = msg.pos_width;
    }
    else {
      resolved.pos_width = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: GetObjectRequest,
  Response: GetObjectResponse,
  md5sum() { return 'e494291815697d37bb402083d19502dd'; },
  datatype() { return 'custom_msgs/GetObject'; }
};
