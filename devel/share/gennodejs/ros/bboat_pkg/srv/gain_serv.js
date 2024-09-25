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

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class gain_servRequest {
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
    // Serializes a message object of type gain_servRequest
    // Serialize message field [req]
    bufferOffset = _serializer.bool(obj.req, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gain_servRequest
    let len;
    let data = new gain_servRequest(null);
    // Deserialize message field [req]
    data.req = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/gain_servRequest';
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
    const resolved = new gain_servRequest(null);
    if (msg.req !== undefined) {
      resolved.req = msg.req;
    }
    else {
      resolved.req = false
    }

    return resolved;
    }
};

class gain_servResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.kp_1 = null;
      this.ki_1 = null;
      this.kd_1 = null;
      this.kp_2 = null;
      this.ki_2 = null;
      this.kd_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('kp_1')) {
        this.kp_1 = initObj.kp_1
      }
      else {
        this.kp_1 = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('ki_1')) {
        this.ki_1 = initObj.ki_1
      }
      else {
        this.ki_1 = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('kd_1')) {
        this.kd_1 = initObj.kd_1
      }
      else {
        this.kd_1 = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('kp_2')) {
        this.kp_2 = initObj.kp_2
      }
      else {
        this.kp_2 = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('ki_2')) {
        this.ki_2 = initObj.ki_2
      }
      else {
        this.ki_2 = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('kd_2')) {
        this.kd_2 = initObj.kd_2
      }
      else {
        this.kd_2 = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gain_servResponse
    // Serialize message field [kp_1]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.kp_1, buffer, bufferOffset);
    // Serialize message field [ki_1]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.ki_1, buffer, bufferOffset);
    // Serialize message field [kd_1]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.kd_1, buffer, bufferOffset);
    // Serialize message field [kp_2]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.kp_2, buffer, bufferOffset);
    // Serialize message field [ki_2]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.ki_2, buffer, bufferOffset);
    // Serialize message field [kd_2]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.kd_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gain_servResponse
    let len;
    let data = new gain_servResponse(null);
    // Deserialize message field [kp_1]
    data.kp_1 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [ki_1]
    data.ki_1 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [kd_1]
    data.kd_1 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [kp_2]
    data.kp_2 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [ki_2]
    data.ki_2 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [kd_2]
    data.kd_2 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/gain_servResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e3308e00b5029dc92caf544598a5631d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64 kp_1
    std_msgs/Float64 ki_1
    std_msgs/Float64 kd_1
    std_msgs/Float64 kp_2
    std_msgs/Float64 ki_2
    std_msgs/Float64 kd_2
    
    
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gain_servResponse(null);
    if (msg.kp_1 !== undefined) {
      resolved.kp_1 = std_msgs.msg.Float64.Resolve(msg.kp_1)
    }
    else {
      resolved.kp_1 = new std_msgs.msg.Float64()
    }

    if (msg.ki_1 !== undefined) {
      resolved.ki_1 = std_msgs.msg.Float64.Resolve(msg.ki_1)
    }
    else {
      resolved.ki_1 = new std_msgs.msg.Float64()
    }

    if (msg.kd_1 !== undefined) {
      resolved.kd_1 = std_msgs.msg.Float64.Resolve(msg.kd_1)
    }
    else {
      resolved.kd_1 = new std_msgs.msg.Float64()
    }

    if (msg.kp_2 !== undefined) {
      resolved.kp_2 = std_msgs.msg.Float64.Resolve(msg.kp_2)
    }
    else {
      resolved.kp_2 = new std_msgs.msg.Float64()
    }

    if (msg.ki_2 !== undefined) {
      resolved.ki_2 = std_msgs.msg.Float64.Resolve(msg.ki_2)
    }
    else {
      resolved.ki_2 = new std_msgs.msg.Float64()
    }

    if (msg.kd_2 !== undefined) {
      resolved.kd_2 = std_msgs.msg.Float64.Resolve(msg.kd_2)
    }
    else {
      resolved.kd_2 = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = {
  Request: gain_servRequest,
  Response: gain_servResponse,
  md5sum() { return '5f632b8cb09f5ccd1ae01e67f49049ea'; },
  datatype() { return 'bboat_pkg/gain_serv'; }
};
