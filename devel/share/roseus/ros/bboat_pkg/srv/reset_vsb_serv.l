;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::reset_vsb_serv)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'reset_vsb_serv (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::RESET_VSB_SERV")
  (make-package "BBOAT_PKG::RESET_VSB_SERV"))
(unless (find-package "BBOAT_PKG::RESET_VSB_SERVREQUEST")
  (make-package "BBOAT_PKG::RESET_VSB_SERVREQUEST"))
(unless (find-package "BBOAT_PKG::RESET_VSB_SERVRESPONSE")
  (make-package "BBOAT_PKG::RESET_VSB_SERVRESPONSE"))

(in-package "ROS")





(defclass bboat_pkg::reset_vsb_servRequest
  :super ros::object
  :slots (_req ))

(defmethod bboat_pkg::reset_vsb_servRequest
  (:init
   (&key
    ((:req __req) nil)
    )
   (send-super :init)
   (setq _req __req)
   self)
  (:req
   (&optional (__req :null))
   (if (not (eq __req :null)) (setq _req __req)) _req)
  (:serialization-length
   ()
   (+
    ;; bool _req
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _req
       (if _req (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _req
     (setq _req (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bboat_pkg::reset_vsb_servResponse
  :super ros::object
  :slots (_resp ))

(defmethod bboat_pkg::reset_vsb_servResponse
  (:init
   (&key
    ((:resp __resp) nil)
    )
   (send-super :init)
   (setq _resp __resp)
   self)
  (:resp
   (&optional (__resp :null))
   (if (not (eq __resp :null)) (setq _resp __resp)) _resp)
  (:serialization-length
   ()
   (+
    ;; bool _resp
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _resp
       (if _resp (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _resp
     (setq _resp (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bboat_pkg::reset_vsb_serv
  :super ros::object
  :slots ())

(setf (get bboat_pkg::reset_vsb_serv :md5sum-) "c42a7ecd5bbbf1290f8a630b218058f2")
(setf (get bboat_pkg::reset_vsb_serv :datatype-) "bboat_pkg/reset_vsb_serv")
(setf (get bboat_pkg::reset_vsb_serv :request) bboat_pkg::reset_vsb_servRequest)
(setf (get bboat_pkg::reset_vsb_serv :response) bboat_pkg::reset_vsb_servResponse)

(defmethod bboat_pkg::reset_vsb_servRequest
  (:response () (instance bboat_pkg::reset_vsb_servResponse :init)))

(setf (get bboat_pkg::reset_vsb_servRequest :md5sum-) "c42a7ecd5bbbf1290f8a630b218058f2")
(setf (get bboat_pkg::reset_vsb_servRequest :datatype-) "bboat_pkg/reset_vsb_servRequest")
(setf (get bboat_pkg::reset_vsb_servRequest :definition-)
      "bool req
---
bool resp

")

(setf (get bboat_pkg::reset_vsb_servResponse :md5sum-) "c42a7ecd5bbbf1290f8a630b218058f2")
(setf (get bboat_pkg::reset_vsb_servResponse :datatype-) "bboat_pkg/reset_vsb_servResponse")
(setf (get bboat_pkg::reset_vsb_servResponse :definition-)
      "bool req
---
bool resp

")



(provide :bboat_pkg/reset_vsb_serv "c42a7ecd5bbbf1290f8a630b218058f2")


