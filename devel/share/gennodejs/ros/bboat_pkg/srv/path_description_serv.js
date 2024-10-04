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

class path_description_servRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type path_description_servRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type path_description_servRequest
    let len;
    let data = new path_description_servRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/path_description_servRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new path_description_servRequest(null);
    return resolved;
    }
};

class path_description_servResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.s = null;
      this.phi_f = null;
      this.curvature = null;
      this.g_c = null;
      this.dx = null;
      this.dy = null;
      this.ddx = null;
      this.ddy = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = [];
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = [];
      }
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = [];
      }
      if (initObj.hasOwnProperty('phi_f')) {
        this.phi_f = initObj.phi_f
      }
      else {
        this.phi_f = [];
      }
      if (initObj.hasOwnProperty('curvature')) {
        this.curvature = initObj.curvature
      }
      else {
        this.curvature = [];
      }
      if (initObj.hasOwnProperty('g_c')) {
        this.g_c = initObj.g_c
      }
      else {
        this.g_c = [];
      }
      if (initObj.hasOwnProperty('dx')) {
        this.dx = initObj.dx
      }
      else {
        this.dx = [];
      }
      if (initObj.hasOwnProperty('dy')) {
        this.dy = initObj.dy
      }
      else {
        this.dy = [];
      }
      if (initObj.hasOwnProperty('ddx')) {
        this.ddx = initObj.ddx
      }
      else {
        this.ddx = [];
      }
      if (initObj.hasOwnProperty('ddy')) {
        this.ddy = initObj.ddy
      }
      else {
        this.ddy = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type path_description_servResponse
    // Serialize message field [x]
    bufferOffset = _arraySerializer.float64(obj.x, buffer, bufferOffset, null);
    // Serialize message field [y]
    bufferOffset = _arraySerializer.float64(obj.y, buffer, bufferOffset, null);
    // Serialize message field [s]
    bufferOffset = _arraySerializer.float64(obj.s, buffer, bufferOffset, null);
    // Serialize message field [phi_f]
    bufferOffset = _arraySerializer.float64(obj.phi_f, buffer, bufferOffset, null);
    // Serialize message field [curvature]
    bufferOffset = _arraySerializer.float64(obj.curvature, buffer, bufferOffset, null);
    // Serialize message field [g_c]
    bufferOffset = _arraySerializer.float64(obj.g_c, buffer, bufferOffset, null);
    // Serialize message field [dx]
    bufferOffset = _arraySerializer.float64(obj.dx, buffer, bufferOffset, null);
    // Serialize message field [dy]
    bufferOffset = _arraySerializer.float64(obj.dy, buffer, bufferOffset, null);
    // Serialize message field [ddx]
    bufferOffset = _arraySerializer.float64(obj.ddx, buffer, bufferOffset, null);
    // Serialize message field [ddy]
    bufferOffset = _arraySerializer.float64(obj.ddy, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type path_description_servResponse
    let len;
    let data = new path_description_servResponse(null);
    // Deserialize message field [x]
    data.x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [y]
    data.y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [s]
    data.s = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [phi_f]
    data.phi_f = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [curvature]
    data.curvature = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [g_c]
    data.g_c = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [dx]
    data.dx = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [dy]
    data.dy = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [ddx]
    data.ddx = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [ddy]
    data.ddy = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.x.length;
    length += 8 * object.y.length;
    length += 8 * object.s.length;
    length += 8 * object.phi_f.length;
    length += 8 * object.curvature.length;
    length += 8 * object.g_c.length;
    length += 8 * object.dx.length;
    length += 8 * object.dy.length;
    length += 8 * object.ddx.length;
    length += 8 * object.ddy.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/path_description_servResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '708adc44b97d996f881dbcca00b89ef2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] x
    float64[] y
    float64[] s
    float64[] phi_f
    float64[] curvature
    float64[] g_c
    float64[] dx
    float64[] dy
    float64[] ddx
    float64[] ddy
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new path_description_servResponse(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = []
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = []
    }

    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = []
    }

    if (msg.phi_f !== undefined) {
      resolved.phi_f = msg.phi_f;
    }
    else {
      resolved.phi_f = []
    }

    if (msg.curvature !== undefined) {
      resolved.curvature = msg.curvature;
    }
    else {
      resolved.curvature = []
    }

    if (msg.g_c !== undefined) {
      resolved.g_c = msg.g_c;
    }
    else {
      resolved.g_c = []
    }

    if (msg.dx !== undefined) {
      resolved.dx = msg.dx;
    }
    else {
      resolved.dx = []
    }

    if (msg.dy !== undefined) {
      resolved.dy = msg.dy;
    }
    else {
      resolved.dy = []
    }

    if (msg.ddx !== undefined) {
      resolved.ddx = msg.ddx;
    }
    else {
      resolved.ddx = []
    }

    if (msg.ddy !== undefined) {
      resolved.ddy = msg.ddy;
    }
    else {
      resolved.ddy = []
    }

    return resolved;
    }
};

module.exports = {
  Request: path_description_servRequest,
  Response: path_description_servResponse,
  md5sum() { return '708adc44b97d996f881dbcca00b89ef2'; },
  datatype() { return 'bboat_pkg/path_description_serv'; }
};
