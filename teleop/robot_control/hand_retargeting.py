"""Hand retargeting configuration loader.

Maps XR hand skeleton data to robot hand joint angles using the
dex-retargeting library.  Each supported hand type has a corresponding
YAML configuration file that describes the kinematic mapping.  This
module loads those configs, builds left/right retargeters, and creates
index arrays that translate retargeting joint order to hardware API
joint order.
"""

from dex_retargeting import RetargetingConfig
from pathlib import Path
import yaml
from enum import Enum
from typing import Any, Dict, List

import logging_mp
logger_mp = logging_mp.getLogger(__name__)


class HandType(Enum):
    """Enumeration of supported robot hand types.

    Each member's value is a relative path to the corresponding YAML
    configuration file used by the dex-retargeting library.

    Attributes:
        INSPIRE_HAND: Inspire DFX dexterous hand config (runtime path).
        INSPIRE_HAND_Unit_Test: Inspire DFX dexterous hand config (unit-test path).
        UNITREE_DEX3: Unitree Dex3 hand config (runtime path).
        UNITREE_DEX3_Unit_Test: Unitree Dex3 hand config (unit-test path).
        BRAINCO_HAND: BrainCo hand config (runtime path).
        BRAINCO_HAND_Unit_Test: BrainCo hand config (unit-test path).
    """

    INSPIRE_HAND = "../assets/inspire_hand/inspire_hand.yml"
    INSPIRE_HAND_Unit_Test = "../../assets/inspire_hand/inspire_hand.yml"
    UNITREE_DEX3 = "../assets/unitree_hand/unitree_dex3.yml"
    UNITREE_DEX3_Unit_Test = "../../assets/unitree_hand/unitree_dex3.yml"
    BRAINCO_HAND = "../assets/brainco_hand/brainco.yml"
    BRAINCO_HAND_Unit_Test = "../../assets/brainco_hand/brainco.yml"


class HandRetargeting:
    """Loads and manages hand retargeting from XR skeleton to robot joints.

    Reads a YAML config for the requested ``HandType``, builds left and
    right ``RetargetingConfig`` retargeters via the dex-retargeting
    library, and constructs index-mapping arrays that reorder retargeting
    joint outputs into the order expected by the hardware API.

    Attributes:
        cfg: Parsed YAML configuration dictionary containing 'left' and
            'right' retargeting sections.
        left_retargeting: Built retargeter instance for the left hand.
        right_retargeting: Built retargeter instance for the right hand.
        left_retargeting_joint_names: Joint name list from the left
            retargeter, in retargeting order.
        right_retargeting_joint_names: Joint name list from the right
            retargeter, in retargeting order.
        left_indices: Human hand link indices targeted by the left
            retargeting optimizer.
        right_indices: Human hand link indices targeted by the right
            retargeting optimizer.
        left_dex_retargeting_to_hardware: Index array mapping left
            retargeting joint order to hardware API joint order.
        right_dex_retargeting_to_hardware: Index array mapping right
            retargeting joint order to hardware API joint order.
    """

    def __init__(self, hand_type: HandType) -> None:
        """Initialise the retargeter for a specific hand type.

        Loads the YAML configuration file for *hand_type*, constructs
        left and right retargeters, extracts joint name lists and human
        link indices, and builds the retargeting-to-hardware index
        mapping arrays for the selected hand model.

        Args:
            hand_type: The robot hand variant to load.  Determines the
                YAML config path and the URDF asset directory.

        Raises:
            FileNotFoundError: If the YAML configuration file does not
                exist at the expected relative path.
            yaml.YAMLError: If the configuration file contains invalid
                YAML syntax.
            ValueError: If the configuration file is missing the
                required ``'left'`` or ``'right'`` keys.
        """
        # Set the URDF asset directory based on the hand type path depth
        if hand_type == HandType.UNITREE_DEX3:
            RetargetingConfig.set_default_urdf_dir('../assets')
        elif hand_type == HandType.UNITREE_DEX3_Unit_Test:
            RetargetingConfig.set_default_urdf_dir('../../assets')
        elif hand_type == HandType.INSPIRE_HAND:
            RetargetingConfig.set_default_urdf_dir('../assets')
        elif hand_type == HandType.INSPIRE_HAND_Unit_Test:
            RetargetingConfig.set_default_urdf_dir('../../assets')
        elif hand_type == HandType.BRAINCO_HAND:
            RetargetingConfig.set_default_urdf_dir('../assets')
        elif hand_type == HandType.BRAINCO_HAND_Unit_Test:
            RetargetingConfig.set_default_urdf_dir('../../assets')

        config_file_path: Path = Path(hand_type.value)

        try:
            with config_file_path.open('r') as f:
                self.cfg: Dict[str, Any] = yaml.safe_load(f)

            if 'left' not in self.cfg or 'right' not in self.cfg:
                raise ValueError("Configuration file must contain 'left' and 'right' keys.")

            # Build left and right retargeters from their config sections
            left_retargeting_config: RetargetingConfig = RetargetingConfig.from_dict(self.cfg['left'])
            right_retargeting_config: RetargetingConfig = RetargetingConfig.from_dict(self.cfg['right'])
            self.left_retargeting = left_retargeting_config.build()
            self.right_retargeting = right_retargeting_config.build()

            # Cache joint names and human target link indices
            self.left_retargeting_joint_names: List[str] = self.left_retargeting.joint_names
            self.right_retargeting_joint_names: List[str] = self.right_retargeting.joint_names
            self.left_indices: List[int] = self.left_retargeting.optimizer.target_link_human_indices
            self.right_indices: List[int] = self.right_retargeting.optimizer.target_link_human_indices

            if hand_type == HandType.UNITREE_DEX3 or hand_type == HandType.UNITREE_DEX3_Unit_Test:
                # In section "Sort by message structure" of https://support.unitree.com/home/en/G1_developer/dexterous_hand
                self.left_dex3_api_joint_names: List[str] = [ 'left_hand_thumb_0_joint', 'left_hand_thumb_1_joint', 'left_hand_thumb_2_joint',
                                                    'left_hand_middle_0_joint', 'left_hand_middle_1_joint',
                                                    'left_hand_index_0_joint', 'left_hand_index_1_joint' ]
                self.right_dex3_api_joint_names: List[str] = [ 'right_hand_thumb_0_joint', 'right_hand_thumb_1_joint', 'right_hand_thumb_2_joint',
                                                    'right_hand_middle_0_joint', 'right_hand_middle_1_joint',
                                                    'right_hand_index_0_joint', 'right_hand_index_1_joint' ]
                # Build index arrays: retargeting joint order -> hardware API joint order
                self.left_dex_retargeting_to_hardware: List[int] = [ self.left_retargeting_joint_names.index(name) for name in self.left_dex3_api_joint_names]
                self.right_dex_retargeting_to_hardware: List[int] = [ self.right_retargeting_joint_names.index(name) for name in self.right_dex3_api_joint_names]

            elif hand_type == HandType.INSPIRE_HAND or hand_type == HandType.INSPIRE_HAND_Unit_Test:
                # "Joint Motor Sequence" of https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand
                self.left_inspire_api_joint_names: List[str] = [ 'L_pinky_proximal_joint', 'L_ring_proximal_joint', 'L_middle_proximal_joint',
                                                       'L_index_proximal_joint', 'L_thumb_proximal_pitch_joint', 'L_thumb_proximal_yaw_joint' ]
                self.right_inspire_api_joint_names: List[str] = [ 'R_pinky_proximal_joint', 'R_ring_proximal_joint', 'R_middle_proximal_joint',
                                                       'R_index_proximal_joint', 'R_thumb_proximal_pitch_joint', 'R_thumb_proximal_yaw_joint' ]
                # Build index arrays: retargeting joint order -> hardware API joint order
                self.left_dex_retargeting_to_hardware: List[int] = [ self.left_retargeting_joint_names.index(name) for name in self.left_inspire_api_joint_names]
                self.right_dex_retargeting_to_hardware: List[int] = [ self.right_retargeting_joint_names.index(name) for name in self.right_inspire_api_joint_names]

            elif hand_type == HandType.BRAINCO_HAND or hand_type == HandType.BRAINCO_HAND_Unit_Test:
                # "Driver Motor ID" of https://www.brainco-hz.com/docs/revolimb-hand/product/parameters.html
                self.left_brainco_api_joint_names: List[str] = [ 'left_thumb_metacarpal_joint', 'left_thumb_proximal_joint', 'left_index_proximal_joint',
                                                       'left_middle_proximal_joint', 'left_ring_proximal_joint', 'left_pinky_proximal_joint' ]
                self.right_brainco_api_joint_names: List[str] = [ 'right_thumb_metacarpal_joint', 'right_thumb_proximal_joint', 'right_index_proximal_joint',
                                                       'right_middle_proximal_joint', 'right_ring_proximal_joint', 'right_pinky_proximal_joint' ]
                # Build index arrays: retargeting joint order -> hardware API joint order
                self.left_dex_retargeting_to_hardware: List[int] = [ self.left_retargeting_joint_names.index(name) for name in self.left_brainco_api_joint_names]
                self.right_dex_retargeting_to_hardware: List[int] = [ self.right_retargeting_joint_names.index(name) for name in self.right_brainco_api_joint_names]

        except FileNotFoundError:
            logger_mp.warning(f"Configuration file not found: {config_file_path}")
            raise
        except yaml.YAMLError as e:
            logger_mp.warning(f"YAML error while reading {config_file_path}: {e}")
            raise
        except Exception as e:
            logger_mp.error(f"An error occurred: {e}")
            raise
