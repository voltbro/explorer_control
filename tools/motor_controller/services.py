import asyncio
import logging

import uavcan.node
from pycyphal.application import Node, make_node, make_transport
from pycyphal.application.register import Natural8, Natural16, Natural32, Real64, ValueProxy
from uavcan.register.Access_1_0 import Access_1_0
from uavcan.register.Name_1_0 import Name_1_0
from uavcan.register.Value_1_0 import Value_1_0
from voltbro.config.six_step.get_1_0 import get_1_0
from voltbro.config.six_step.log_1_0 import log_1_0
from voltbro.config.six_step.set_1_0 import set_1_0

from network_controller import db_path, id_from_name, srv_port_from_name
from utils import singleton

logger = logging.getLogger("services")

PASSIVE = ('get config', 'log control')
ACTIVE = ('set config',)

NAME_MAP = {
    'get config': get_1_0,
    'set config': set_1_0,
    'log control': log_1_0,
}


class ServicesProxy:
    def __init__(self, collection: 'ServicesCollection'):
        self.k1 = None
        self.k2 = None
        self.collection = collection

    def __getitem__(self, key):
        service = None

        if self.k1:
            if self.k2:
                raise RuntimeError("Proxy exhausted")
            if key not in (*PASSIVE, *ACTIVE):
                raise KeyError(f"Unknown service name {key}")
            self.k2 = key
            service = self.collection.services[self.k1][key]
        else:
            self.k1 = key

        if service is None:
            return self

        async def cyphal_caller(*args):
            result = await service.call(NAME_MAP[key].Request(*args))
            return result[0] if result is not None else None

        return cyphal_caller


@singleton
class ServicesCollection:
    REGISTER_FILE = db_path / "services_proxy.db"

    def __init__(self) -> None:
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name="org.voltbro.simple_control",
        )
        node_id = id_from_name(ServicesCollection.__name__)
        transport = make_transport(
            {
                "uavcan.can.iface": ValueProxy("socketcan:can0"),
                "uavcan.node.id": ValueProxy(Natural16([node_id])),
                "uavcan.can.mtu": ValueProxy(Natural16([64])),
                "uavcan.can.bitrate": ValueProxy(Natural32([1000000, 8000000])),
            }
        )
        self._node: Node = make_node(
            node_info,
            self.REGISTER_FILE,
            transport=transport,
        )

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = node_id

        self.services = {}
        for motor_id in (2, 4, 8, 16):
            services = {}
            for service in ('get config', 'set config', 'log control'):
                s = self._node.make_client(NAME_MAP[service], motor_id, srv_port_from_name(service))
                s.response_timeout = 1
                services[service] = s
            self.services[motor_id] = services

        self.registers = {}
        for motor_id in (2, 4, 8, 16):
            self.registers[motor_id] = self._node.make_client(Access_1_0, motor_id)

        self.left_regs = [self.registers[t_id] for t_id in (2, 8)]
        self.right_regs = [self.registers[t_id] for t_id in (4, 16)]
        for s in (*self.left_regs, *self.right_regs):
            s.response_timeout = 1

    def start(self):
        self._node.start()

    def __getitem__(self, key):
        proxy = ServicesProxy(self)
        return proxy[key]

    def close(self) -> None:
        self._node.close()

    async def set_speed_by_id(self, node_id, speed) -> None:
        return await self.registers[node_id].call(
            Access_1_0.Request(Name_1_0("motor.speed"), Value_1_0(real64=Real64([speed])))
        )

    def set_left_speed(self, speed) -> None:
        return asyncio.gather(
            *[
                serv.call(
                    Access_1_0.Request(Name_1_0("motor.speed"), Value_1_0(real64=Real64([speed])))
                )
                for serv in self.left_regs
            ]
        )

    def set_right_speed(self, speed) -> None:
        return asyncio.gather(
            *[
                serv.call(
                    Access_1_0.Request(Name_1_0("motor.speed"), Value_1_0(real64=Real64([speed])))
                )
                for serv in self.right_regs
            ]
        )

    async def set_speed(self, speed) -> None:
        return await asyncio.gather(
            self.set_left_speed(-speed),
            self.set_right_speed(speed),
        )

    async def set_turn(self, turn) -> None:
        return await asyncio.gather(
            self.set_left_speed(turn),
            self.set_right_speed(turn),
        )

    async def set_state_all(self, state) -> None:
        return await asyncio.gather(
            *[
                s.call(
                    Access_1_0.Request(
                        Name_1_0("motor.is_on"), Value_1_0(natural8=Natural8([state]))
                    )
                )
                for s in self.left_regs
            ],
            *[
                s.call(
                    Access_1_0.Request(
                        Name_1_0("motor.is_on"), Value_1_0(natural8=Natural8([state]))
                    )
                )
                for s in self.right_regs
            ],
        )

    async def set_state(self, motor_id, state) -> None:
        return await self.registers[motor_id].call(
            Access_1_0.Request(Name_1_0("motor.is_on"), Value_1_0(natural8=Natural8([state])))
        )

    async def get_state(self, motor_id) -> None:
        result = await self.registers[motor_id].call(Access_1_0.Request(Name_1_0("motor.is_on")))
        if result is None:
            return None

        return result[0].value.natural8.value[0]


async def main():
    services_collection = ServicesCollection()
    services_collection.start()
    log = await services_collection[35]['log control']()
    print(log.log_item.I_A.value)  # noqa: T201
    services_collection.close()


if __name__ == "__main__":
    asyncio.run(main())
