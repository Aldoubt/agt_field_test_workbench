"""Bounded retries for idempotent parameter reads, never for mutating services."""
import time


def read_parameters(node, name, keys, *, health_check=lambda: None,
                    discovery_timeout=8., response_timeout=2., attempts=3):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    endpoint='/'+name.strip('/')+'/get_parameters'
    client=node.create_client(GetParameters,endpoint)
    try:
        deadline=time.monotonic()+discovery_timeout
        while not client.service_is_ready():
            health_check()
            if time.monotonic()>=deadline:
                raise ValueError(endpoint+' service unavailable')
            rclpy.spin_once(node,timeout_sec=.05)
        for attempt in range(1,attempts+1):
            health_check()
            future=client.call_async(GetParameters.Request(names=keys))
            deadline=time.monotonic()+response_timeout
            try:
                while not future.done() and time.monotonic()<deadline:
                    health_check()
                    rclpy.spin_once(node,timeout_sec=.05)
                if future.done() and future.result() is not None:
                    values=future.result().values
                    if len(values)!=len(keys):
                        raise ValueError(endpoint+' returned an invalid parameter count')
                    return {key:parameter_value_to_python(value) for key,value in zip(keys,values)}
            finally:
                # A late reply from a lost/expired attempt cannot complete the
                # next request. Keep the discovered client for subsequent reads.
                client.remove_pending_request(future)
                if not future.done():
                    future.cancel()
            if attempt<attempts:
                print(f'{endpoint} 参数响应超时，重试 {attempt+1}/{attempts}',flush=True)
        raise ValueError(endpoint+f' 参数读取失败（{attempts} 次有界只读尝试）')
    finally:
        node.destroy_client(client)
