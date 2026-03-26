"""Motion mode switching for Unitree robots.

Controls transitions between debug mode (direct joint control) and
AI/locomotion modes via the Unitree SDK.  ``MotionSwitcher`` wraps the
low-level ``MotionSwitcherClient`` for mode transitions, while
``LocoClientWrapper`` wraps ``LocoClient`` for locomotion commands such
as velocity moves and entering damp mode.
"""

# for motion switcher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
# for loco client
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
import time
from typing import Optional, Tuple, Dict


# MotionSwitcher used to switch mode between debug mode and ai mode
class MotionSwitcher:
    """Wrapper around ``MotionSwitcherClient`` for mode transitions.

    Provides helper methods to enter *debug mode* (releases all active
    locomotion modes so that joints can be commanded directly) and to
    exit back to *AI mode* (re-enables the locomotion controller).

    Attributes:
        msc: The underlying ``MotionSwitcherClient`` instance.
    """

    def __init__(self) -> None:
        """Initialise the motion switcher client.

        Creates and initialises a ``MotionSwitcherClient`` with a
        1-second timeout.
        """
        self.msc: MotionSwitcherClient = MotionSwitcherClient()
        self.msc.SetTimeout(1.0)
        self.msc.Init()

    def Enter_Debug_Mode(self) -> Tuple[Optional[int], Optional[Dict]]:
        """Transition the robot into debug mode.

        Polls the current mode and repeatedly calls ``ReleaseMode``
        until no locomotion mode is active.  Each release attempt is
        followed by a 1-second sleep to allow the controller to
        transition.

        Returns:
            A tuple ``(status, result)`` from the last
            ``CheckMode`` call.  Returns ``(None, None)`` if an
            exception occurs during the transition.
        """
        try:
            status, result = self.msc.CheckMode()
            # Keep releasing until no active mode remains
            while result['name']:
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)
            return status, result
        except Exception as e:
            return None, None

    def Exit_Debug_Mode(self) -> Tuple[Optional[int], Optional[Dict]]:
        """Transition the robot out of debug mode back to AI mode.

        Selects the ``'ai'`` locomotion mode via the motion switcher.

        Returns:
            A tuple ``(status, result)`` from ``SelectMode``.  Returns
            ``(None, None)`` if an exception occurs.
        """
        try:
            status, result = self.msc.SelectMode(nameOrAlias='ai')
            return status, result
        except Exception as e:
            return None, None


class LocoClientWrapper:
    """Wrapper around ``LocoClient`` for locomotion commands.

    Provides simplified methods for sending velocity commands and
    entering damp (passive compliance) mode.

    Attributes:
        client: The underlying ``LocoClient`` instance.
    """

    def __init__(self) -> None:
        """Initialise the locomotion client.

        Creates and initialises a ``LocoClient`` with a very short
        timeout (0.0001 s) suitable for non-blocking calls.
        """
        self.client: LocoClient = LocoClient()
        self.client.SetTimeout(0.0001)
        self.client.Init()

    def Enter_Damp_Mode(self) -> None:
        """Command the robot into damp (passive compliance) mode.

        In damp mode the robot's joints become compliant and no active
        control is applied, allowing external forces to move the limbs.
        """
        self.client.Damp()

    def Move(self, vx: float, vy: float, vyaw: float) -> None:
        """Send a velocity command to the locomotion controller.

        Args:
            vx: Forward/backward velocity in m/s.
            vy: Left/right lateral velocity in m/s.
            vyaw: Yaw rotational velocity in rad/s.
        """
        self.client.Move(vx, vy, vyaw, continous_move=False)


if __name__ == '__main__':
    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation
    ms = MotionSwitcher()
    status, result = ms.Enter_Debug_Mode()
    print("Enter debug mode:", status, result)
    time.sleep(5)
    status, result = ms.Exit_Debug_Mode()
    print("Exit debug mode:", status, result)
    time.sleep(2)
