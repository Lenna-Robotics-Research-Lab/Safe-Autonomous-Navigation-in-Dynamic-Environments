#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cvxpy as cp

from erl_clf_cbf_controller.msg import ClfCbfRequest, ClfCbfResult
from erl_clf_cbf_controller.clf_cbf_controller import ClfCbfController
from erl_clf_cbf_controller.clf_only_controller import ClfQPController


from erl_clf_cbf_controller.clf_cbf_socp_controller_gp_map import ClfCbfSOCP_GP_MAP

from erl_clf_cbf_controller.clf_cbf_drccp_dynamic_controller import ClfCbfDrccp_dynamic_Controller

import os
import json

class CvxpySolverNode(object):
    def __init__(self):
        rospy.init_node('clf_cbf_cvxpy_solver')

        # read parameters or use defaults
        self.controller = None
        self.controller_type = rospy.get_param("~controller_type", "baseline_clf_cbf_qp")
        self.load_controller_config()
        self.req_sub = rospy.Subscriber('clf_cbf/request', ClfCbfRequest,
                                        self.req_cb, queue_size=10)
        self.res_pub = rospy.Publisher('clf_cbf/result', ClfCbfResult,
                                       queue_size=10)
    def load_controller_config(self):
        # Load the configuration file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "controller_config.json")
        with open(config_path, "r") as file:
            configs = json.load(file)

        # Determine which controller configuration to use
        controller_type = self.controller_type

        if controller_type not in configs:
            raise ValueError("Unknown controller configuration: {}".format(controller_type))

        self.config = configs[controller_type]

        # Assigning new variables from the selected config
        params = self.config["parameters"]
        self._wheel_offset = params.get("wheel_offset", 0.08)  # Default to 0.08 if not set
        self._cbf_rate = params.get("cbf_rate", 0.4)  # Default to 0.4 if not set
        self.noise_level = params.get("noise_level", 0.01)  # Default to 0.01 if not set
        
        # Initialize the controller based on the controller type
        self.init_core()

    def init_core(self):
        """
        Init unicycle controller. For each controller the interface might be different.
        More details in controller itself.
        """
        # Initialize the controller based on the configuration
        controller_type = self.controller_type
        params = self.config["parameters"]

        # if controller_type not in [
        #     "baseline_clf_cbf_qp",
        #     "clf_qp_only",
        #     "robust_cbf_socp",
        #     "gp_cbf_socp",
        #     "drccp"
        # ]:
        #     raise ValueError("Unknown controller configuration: %s" % controller_type)

        if controller_type == "baseline_clf_cbf_qp":
            self.controller = ClfCbfController(**params)
        elif controller_type == "clf_qp_only":
            self.controller = ClfQPController(**params)
        elif controller_type == "robust_cbf_socp":
            self.controller = ClfCbf_Robust_SOCP_Controller(**params)
        elif controller_type == "gp_cbf_socp":
            self.controller = ClfCbfSOCP_GP_MAP(**params)
        elif controller_type == "drccp":
            self.controller = ClfCbfDrccp_dynamic_Controller(**params)
        else:
            raise ValueError("Unknown controller type specified.")


    def req_cb(self, msg):
        # build numpy arrays
        rbt_pose = np.array([msg.x, msg.y, msg.theta])
        gamma_s = np.array([msg.gamma_x, msg.gamma_y])
        cbf_h_val = msg.h
        cbf_h_grad = np.array([msg.h_grad_x, msg.h_grad_y])

        res = ClfCbfResult()
        try:
            u = self.controller.generate_controller(rbt_pose, gamma_s,
                                                    cbf_h_val, cbf_h_grad)
            res.v = float(u[0])
            res.omega = float(u[1])
            res.status = 0 
        except Exception as e:
            rospy.logwarn("cvxpy solver exception: %s", str(e))
            res.v = 0.0
            res.omega = 0.0
            res.status = 1

        self.res_pub.publish(res)


if __name__ == '__main__':
    node = CvxpySolverNode()
    rospy.spin()
