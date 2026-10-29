"""Correlate a UI motion request with backend evidence, without retrying commands."""
from copy import deepcopy
from uuid import uuid4


class ActionTracker:
    def __init__(self):
        self.motion = None
        self.stop = None

    @staticmethod
    def _new(kind):
        return dict(request_id=str(uuid4()), kind=kind, state='SENDING', terminal=False,
                    confirmed=False, action_id=None, bridge_id=None, seq=-1, reason='')

    @property
    def busy(self):
        return any(item and not (item['confirmed'] and item['terminal']) for item in (self.motion,self.stop))

    def begin(self, kind):
        if kind=='stop':
            self.stop=self._new(kind)
            payload={'request_id':self.stop['request_id']}
            if self.motion and not (self.motion['confirmed'] and self.motion['terminal']):
                payload['target_request_id']=self.motion['request_id']
            return payload
        if self.busy:raise ValueError('已有未确认完成的运动请求')
        self.motion=self._new(kind);self.stop=None
        return {'request_id':self.motion['request_id']}

    def _merge(self, action, *, confirmed=False, bridge_id=None):
        if not isinstance(action,dict):return False
        ident=action.get('request_id')
        local=next((item for item in (self.stop,self.motion) if item and item['request_id']==ident),None)
        if local is None:return False
        remote_bridge=action.get('bridge_id') or bridge_id
        if local['bridge_id'] and remote_bridge!=local['bridge_id']:return False
        if local['action_id'] and action.get('action_id')!=local['action_id']:return False
        seq=action.get('seq')
        if not isinstance(seq,int) or isinstance(seq,bool) or seq<local['seq']:return False
        if local['confirmed'] and local['terminal'] and not action.get('terminal'):return False
        if not remote_bridge or not action.get('action_id'):return False
        for key in ('action_id','request_id','kind','state','terminal','reason','seq','updated_at'):
            if key in action:local[key]=deepcopy(action[key])
        local['bridge_id']=remote_bridge
        local['confirmed']=bool(local['confirmed'] or confirmed)
        if local is self.stop and local['confirmed'] and local['terminal'] and local['state']=='DISPATCHED':
            # This only confirms dispatch/cancel processing, never physical standstill.
            if self.motion and not (self.motion['confirmed'] and self.motion['terminal']):
                self.motion.update(state='CANCELED',terminal=True,confirmed=True,
                                   reason='停止已下发，原请求已作废；实际停稳请核对底盘')
        return True

    def acknowledge(self, response):
        action=response.get('action') if isinstance(response,dict) else None
        return self._merge(action,bridge_id=response.get('bridge_id') if isinstance(response,dict) else None)

    def observe(self, status):
        if not isinstance(status,dict):return
        bridge_id=status.get('bridge_id')
        for item in (self.motion,self.stop):
            if item and not item['terminal'] and item['bridge_id'] and bridge_id and item['bridge_id']!=bridge_id:
                item.update(state='UNKNOWN',reason='ROS 桥已重启，原动作结果未知；请停止并重新确认')
        records=list(status.get('recent') or [])
        records.extend(status.get(key) for key in ('last','current'))
        for action in records:self._merge(action,confirmed=True,bridge_id=bridge_id)

    def failed_transport(self, kind, reason):
        item=self.stop if kind=='stop' else self.motion
        if item and not item['terminal']:
            item.update(state='UNKNOWN',reason='请求结果未知；不自动重发。'+str(reason))

    def summary(self):
        item=self.stop or self.motion
        if not item:return ''
        labels={'SENDING':'正在发送','REQUESTED':'已受理，等待执行','ARMING':'等待底盘使能','ACCEPTED':'已接受',
                'RUNNING':'执行中','SUCCEEDED':'已完成','FAILED':'失败','REJECTED':'已拒绝',
                'CANCELED':'已取消','CANCELLED':'已取消','CANCEL_REQUESTED':'等待取消结果',
                'STOP_REQUESTED':'停止请求已受理','DISPATCHED':'停止已下发（需核对实际停稳）','UNKNOWN':'结果未知'}
        detail=labels.get(item['state'],item['state'])
        if item['terminal'] and not item['confirmed']:detail+=' · 等待状态确认'
        return detail+' · '+item['request_id'][:8]+(' · '+item['reason'] if item.get('reason') else '')

    def snapshot(self):
        return deepcopy(dict(motion=self.motion,stop=self.stop))
