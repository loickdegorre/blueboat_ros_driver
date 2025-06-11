
(cl:in-package :asdf)

(defsystem "bboat_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "current_target_serv" :depends-on ("_package_current_target_serv"))
    (:file "_package_current_target_serv" :depends-on ("_package"))
    (:file "gain_serv" :depends-on ("_package_gain_serv"))
    (:file "_package_gain_serv" :depends-on ("_package"))
    (:file "lambert_ref_serv" :depends-on ("_package_lambert_ref_serv"))
    (:file "_package_lambert_ref_serv" :depends-on ("_package"))
    (:file "mode_serv" :depends-on ("_package_mode_serv"))
    (:file "_package_mode_serv" :depends-on ("_package"))
    (:file "next_target_serv" :depends-on ("_package_next_target_serv"))
    (:file "_package_next_target_serv" :depends-on ("_package"))
    (:file "path_description_serv" :depends-on ("_package_path_description_serv"))
    (:file "_package_path_description_serv" :depends-on ("_package"))
    (:file "reset_lamb_serv" :depends-on ("_package_reset_lamb_serv"))
    (:file "_package_reset_lamb_serv" :depends-on ("_package"))
    (:file "reset_vsb_serv" :depends-on ("_package_reset_vsb_serv"))
    (:file "_package_reset_vsb_serv" :depends-on ("_package"))
    (:file "traj_serv" :depends-on ("_package_traj_serv"))
    (:file "_package_traj_serv" :depends-on ("_package"))
  ))