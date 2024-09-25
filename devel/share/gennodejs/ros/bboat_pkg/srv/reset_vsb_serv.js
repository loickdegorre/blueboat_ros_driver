// Auto-generated. Do not edit!

// (in-package bboat_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class reset_vsb_servRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.req = null;
    }
    else {
      if (initObj.hasOwnProperty('req')) {
        this.req = initObj.req
      }
      else {
        this.req = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reset_vsb_servRequest
    // Serialize message field [req]
    bufferOffset = _serializer.bool(obj.req, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reset_vsb_servRequest
    let len;
    let data = new reset_vsb_servRequest(null);
    // Deserialize message field [req]
    data.req = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/reset_vsb_servRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be3c44e19d0c6b00b25e356c69155e2a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool req
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new reset_vsb_servRequest(null);
    if (msg.req !== undefined) {
      resolved.req = msg.req;
    }
    else {
      resolved.req = false
    }

    return resolved;
    }
};

class reset_vsb_servResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.resp = null;
    }
    else {
      if (initObj.hasOwnProperty('resp')) {
        this.resp = initObj.resp
      }
      else {
        this.resp = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reset_vsb_servResponse
    // Serialize message field [resp]
    bufferOffset = _serializer.bool(obj.resp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reset_vsb_servResponse
    let len;
    let data = new reset_vsb_servResponse(null);
    // Deserialize message field [resp]
    data.resp = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/reset_vsb_servResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e3a98d806e1c4fee01a17d597e27ea8e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool resp
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new reset_vsb_servResponse(null);
    if (msg.resp !== undefined) {
      resolved.resp = msg.resp;
    }
    else {
      resolved.resp = false
    }

    return resolved;
    }
};

module.exports = {
  Request: reset_vsb_servRequest,
  Response: reset_vsb_servResponse,
  md5sum() { return 'c42a7ecd5bbbf1290f8a630b218058f2'; },
  datatype() { return 'bboat_pkg/reset_vsb_serv'; }
};
