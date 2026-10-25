"""Isolated service wire test; no localization pose or chassis/velocity publishers."""
import os
if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
    raise SystemExit('Use an isolated ROS_DOMAIN_ID, never domain 0')

import threading
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import Trigger
from workbench.field_trial import request_relocalization


def main():
    context = Context();rclpy.init(context=context)
    server = Node('field_trial_request_test_services', context=context)
    calls = []
    def set_budget(request, response):
        assert len(request.parameters) == 1
        parameter = request.parameters[0]
        assert parameter.name == 'sdk_timeout_sec' and parameter.value.double_value == 60.
        calls.append('budget')
        response.result.successful = len(calls) == 1
        response.result.reason = 'reject second update for failure-path test'
        return response
    def request(request, response):
        assert calls == ['budget']
        calls.append('request');response.success = True;response.message = 'request accepted'
        return response
    server.create_service(SetParametersAtomically, '/agt_global_relocalization/set_parameters_atomically', set_budget)
    server.create_service(Trigger, '/agt/localization/relocalize', request)
    executor = SingleThreadedExecutor(context=context);executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True);thread.start()
    try:
        assert request_relocalization(60.) == 0
        assert calls == ['budget', 'request']
        try:
            request_relocalization(60.)
        except ValueError as error:
            assert 'rejected compute budget' in str(error)
        else:
            raise AssertionError('rejected parameter update must stop the request')
        assert calls == ['budget', 'request', 'budget']
        print('PASS: set compute budget before request; rejected update prevents request')
    finally:
        executor.shutdown();thread.join(timeout=3);server.destroy_node();rclpy.shutdown(context=context)


if __name__ == '__main__':
    main()
