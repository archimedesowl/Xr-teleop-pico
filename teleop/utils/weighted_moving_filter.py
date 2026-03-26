"""Weighted moving average filter for smoothing noisy signal data.

Used to smooth IK solutions and gripper commands in the teleoperation
pipeline.  The filter maintains a sliding window of recent data points
and applies a weighted convolution to produce a smoothed output.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List


class WeightedMovingFilter:
    """Weighted moving average filter over a fixed-size data vector.

    Maintains an internal queue of recent data vectors and applies a
    1-D weighted convolution independently to each element of the
    vector.  Duplicate consecutive data points are automatically
    discarded to avoid biasing the filter.

    Attributes:
        _window_size: Number of data points in the sliding window
            (equal to the length of *weights*).
        _weights: 1-D array of convolution weights.  Must sum to 1.0.
        _data_size: Dimensionality of each data vector.
        _filtered_data: Most recent filtered output vector.
        _data_queue: Internal FIFO of raw data vectors (list of arrays).
    """

    def __init__(self, weights: np.ndarray, data_size: int = 14) -> None:
        """Initialise the filter.

        Args:
            weights: 1-D array-like of convolution weights.  The length
                determines the window size.  The values **must** sum to
                1.0.
            data_size: Number of elements in each data vector.

        Raises:
            AssertionError: If the weights do not sum to 1.0.
        """
        self._window_size: int = len(weights)
        self._weights: np.ndarray = np.array(weights)
        assert np.isclose(np.sum(self._weights), 1.0), "[WeightedMovingFilter] the sum of weights list must be 1.0!"
        self._data_size: int = data_size
        self._filtered_data: np.ndarray = np.zeros(self._data_size)
        self._data_queue: List[np.ndarray] = []

    def _apply_filter(self) -> np.ndarray:
        """Convolve the data queue with the weight vector.

        If fewer data points than the window size have been collected
        the most recent raw value is returned without filtering.

        Returns:
            Filtered data vector of shape ``(data_size,)``.
        """
        if len(self._data_queue) < self._window_size:
            # Not enough data yet — return the latest raw sample
            return self._data_queue[-1]

        data_array: np.ndarray = np.array(self._data_queue)
        temp_filtered_data: np.ndarray = np.zeros(self._data_size)
        # Apply weighted convolution independently per channel
        for i in range(self._data_size):
            temp_filtered_data[i] = np.convolve(data_array[:, i], self._weights, mode='valid')[-1]

        return temp_filtered_data

    def add_data(self, new_data: np.ndarray) -> None:
        """Add a data point to the queue and update the filter output.

        Duplicate consecutive values (exact equality) are silently
        discarded.  When the queue exceeds the window size the oldest
        entry is dropped.

        Args:
            new_data: Data vector of length ``data_size``.

        Raises:
            AssertionError: If ``len(new_data) != data_size``.
        """
        assert len(new_data) == self._data_size

        if len(self._data_queue) > 0 and np.array_equal(new_data, self._data_queue[-1]):
            return  # skip duplicate data

        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)

        self._data_queue.append(new_data)
        self._filtered_data = self._apply_filter()

    @property
    def filtered_data(self) -> np.ndarray:
        """Return the most recently computed filtered data vector.

        Returns:
            Numpy array of shape ``(data_size,)`` with the latest
            smoothed values.
        """
        return self._filtered_data


def visualize_filter_comparison(filter_params: List[np.ndarray], steps: int) -> None:
    """Generate a side-by-side comparison plot for multiple filter configs.

    Creates a synthetic noisy sine-wave dataset and runs each weight
    configuration through a ``WeightedMovingFilter``.  Two subplots per
    configuration show (1) an unfiltered channel and (2) a filtered
    channel so the smoothing effect can be visually assessed.

    This is a standalone test/visualisation function intended for
    interactive development, not for production use.

    Args:
        filter_params: List of 1-D weight arrays, one per filter
            configuration to compare.
        steps: Number of time steps in the synthetic dataset.
    """
    import time
    t: np.ndarray = np.linspace(0, 4 * np.pi, steps)
    # Synthetic sin wave with additive Gaussian noise, shape: [steps, 35]
    original_data: np.ndarray = np.array([np.sin(t + i) + np.random.normal(0, 0.2, len(t)) for i in range(35)]).T  # sin wave with noise, shape is [len(t), 35]

    plt.figure(figsize=(14, 10))

    for idx, weights in enumerate(filter_params):
        filter = WeightedMovingFilter(weights, 14)
        data_2b_filtered: np.ndarray = original_data.copy()
        filtered_data: List[np.ndarray] = []

        time1: float = time.time()

        for i in range(steps):
            filter.add_data(data_2b_filtered[i][13:27])            # step i, columns 13 to 26 (total:14)
            data_2b_filtered[i][13:27] = filter.filtered_data
            filtered_data.append(data_2b_filtered[i])

        time2: float = time.time()
        print(f"filter_params:{filter_params[idx]}, time cosume:{time2 - time1}")

        filtered_data_arr: np.ndarray = np.array(filtered_data)

        # col0 should not 2b filtered
        plt.subplot(len(filter_params), 2, idx * 2 + 1)
        plt.plot(filtered_data_arr[:, 0], label=f'Filtered (Window {filter._window_size})')
        plt.plot(original_data[:, 0], 'r--', label='Original', alpha=0.5)
        plt.title(f'Joint 1 - Should not to be filtered.')
        plt.xlabel('Step')
        plt.ylabel('Value')
        plt.legend()

        # col13 should 2b filtered
        plt.subplot(len(filter_params), 2, idx * 2 + 2)
        plt.plot(filtered_data_arr[:, 13], label=f'Filtered (Window {filter._window_size})')
        plt.plot(original_data[:, 13], 'r--', label='Original', alpha=0.5)
        plt.title(f'Joint 13 - Window {filter._window_size}, Weights {weights}')
        plt.xlabel('Step')
        plt.ylabel('Value')
        plt.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # windows_size and weights
    filter_params = [
        (np.array([0.7, 0.2, 0.1])),
        (np.array([0.5, 0.3, 0.2])),
        (np.array([0.4, 0.3, 0.2, 0.1])),
    ]

    visualize_filter_comparison(filter_params, steps = 100)
