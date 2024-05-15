;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::reset_lamb_serv)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'reset_lamb_serv (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::RESET_LAMB_SERV")
  (make-package "BBOAT_PKG::RESET_LAMB_SERV"))
(unless (find-package "BBOAT_PKG::RESET_LAMB_SERVREQUEST")
  (make-package "BBOAT_PKG::RESET_LAMB_SERVREQUEST"))
(unless (find-package "BBOAT_PKG::RESET_LAMB_SERVRESPONSE")
  (make-package "BBOAT_PKG::RESET_LAMB_SERVRESPONSE"))

(in-package "ROS")





(defclass bboat_pkg::reset_lamb_servRequest
  :super ros::object
  :slots (_req ))

(defmethod bboat_pkg::reset_lamb_servRequest
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

(defclass bboat_pkg::reset_lamb_servResponse
  :super ros::object
  :slots (_success ))

(defmethod bboat_pkg::reset_lamb_servResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass bboat_pkg::reset_lamb_serv
  :super ros::object
  :slots ())

(setf (get bboat_pkg::reset_lamb_serv :md5sum-) "e188e0aaebb633ed14c21333921ef038")
(setf (get bboat_pkg::reset_lamb_serv :datatype-) "bboat_pkg/reset_lamb_serv")
(setf (get bboat_pkg::reset_lamb_serv :request) bboat_pkg::reset_lamb_servRequest)
(setf (get bboat_pkg::reset_lamb_serv :response) bboat_pkg::reset_lamb_servResponse)

(defmethod bboat_pkg::reset_lamb_servRequest
  (:response () (instance bboat_pkg::reset_lamb_servResponse :init)))

(setf (get bboat_pkg::reset_lamb_servRequest :md5sum-) "e188e0aaebb633ed14c21333921ef038")
(setf (get bboat_pkg::reset_lamb_servRequest :datatype-) "bboat_pkg/reset_lamb_servRequest")
(setf (get bboat_pkg::reset_lamb_servRequest :definition-)
      "bool req
---
bool success
")

(setf (get bboat_pkg::reset_lamb_servResponse :md5sum-) "e188e0aaebb633ed14c21333921ef038")
(setf (get bboat_pkg::reset_lamb_servResponse :datatype-) "bboat_pkg/reset_lamb_servResponse")
(setf (get bboat_pkg::reset_lamb_servResponse :definition-)
      "bool req
---
bool success
")



(provide :bboat_pkg/reset_lamb_serv "e188e0aaebb633ed14c21333921ef038")


