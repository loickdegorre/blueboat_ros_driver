;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::mode_serv)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'mode_serv (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::MODE_SERV")
  (make-package "BBOAT_PKG::MODE_SERV"))
(unless (find-package "BBOAT_PKG::MODE_SERVREQUEST")
  (make-package "BBOAT_PKG::MODE_SERVREQUEST"))
(unless (find-package "BBOAT_PKG::MODE_SERVRESPONSE")
  (make-package "BBOAT_PKG::MODE_SERVRESPONSE"))

(in-package "ROS")





(defclass bboat_pkg::mode_servRequest
  :super ros::object
  :slots (_req ))

(defmethod bboat_pkg::mode_servRequest
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

(defclass bboat_pkg::mode_servResponse
  :super ros::object
  :slots (_mode _mission ))

(defmethod bboat_pkg::mode_servResponse
  (:init
   (&key
    ((:mode __mode) "")
    ((:mission __mission) "")
    )
   (send-super :init)
   (setq _mode (string __mode))
   (setq _mission (string __mission))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:mission
   (&optional __mission)
   (if __mission (setq _mission __mission)) _mission)
  (:serialization-length
   ()
   (+
    ;; string _mode
    4 (length _mode)
    ;; string _mission
    4 (length _mission)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; string _mission
       (write-long (length _mission) s) (princ _mission s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _mission
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mission (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass bboat_pkg::mode_serv
  :super ros::object
  :slots ())

(setf (get bboat_pkg::mode_serv :md5sum-) "a6ec36370aeacdd14ee457803ad5dd8a")
(setf (get bboat_pkg::mode_serv :datatype-) "bboat_pkg/mode_serv")
(setf (get bboat_pkg::mode_serv :request) bboat_pkg::mode_servRequest)
(setf (get bboat_pkg::mode_serv :response) bboat_pkg::mode_servResponse)

(defmethod bboat_pkg::mode_servRequest
  (:response () (instance bboat_pkg::mode_servResponse :init)))

(setf (get bboat_pkg::mode_servRequest :md5sum-) "a6ec36370aeacdd14ee457803ad5dd8a")
(setf (get bboat_pkg::mode_servRequest :datatype-) "bboat_pkg/mode_servRequest")
(setf (get bboat_pkg::mode_servRequest :definition-)
      "bool req
---
string mode
string mission

")

(setf (get bboat_pkg::mode_servResponse :md5sum-) "a6ec36370aeacdd14ee457803ad5dd8a")
(setf (get bboat_pkg::mode_servResponse :datatype-) "bboat_pkg/mode_servResponse")
(setf (get bboat_pkg::mode_servResponse :definition-)
      "bool req
---
string mode
string mission

")



(provide :bboat_pkg/mode_serv "a6ec36370aeacdd14ee457803ad5dd8a")


