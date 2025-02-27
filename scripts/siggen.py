# Building a class Signal for better use.

import numpy as np

class Signal:
    """
    Generate sinusoidal signals with specific ampltiudes, frequencies, duration,
    sampling rate, and phase.

    Example:
      signal = Signal(amplitude=10, sampling_rate=2000.0)
      sine = signal.sine()
      cosine = signal.cosine()
    """

    def __init__(self, amplitude=100, frequency=400, sampling_rate=16000, phase=0):
        """
        Initialize the Signal class.

        Args:
            amplitude (int16): The amplitude of the signal
            frequency (int16): The frequency of the signal Hz
            sampling_rate (int): The sampling per second of the signal
            phase (float): The phase of the signal in radians
                            As the generator (sine) is called, phase advances

        Additional parameters,which are required to generate the signal, are
        calculated and defined to be initialized here too:
            time_step (float): 1.0/sampling_rate
        """
        self.amplitude = amplitude
        self.frequency = frequency
        self.sampling_rate = sampling_rate
        self.phase = phase
        self.time_step = 1.0 / self.sampling_rate
        self.samples_in_a_cycle = int(sampling_rate/frequency)
        #print("samples_in_a_cycle: ",self.samples_in_a_cycle)
        _data = amplitude * np.sin(2 * np.pi * frequency * np.arange(self.samples_in_a_cycle) * self.time_step + phase)
        self.data = _data.astype('int16')
        self.ptr = 0

    # Generate sine wave
    def sine(self, num_samples):
        """
        Method of Signal
        Called with:
            num_samples - number of samples to return
        Returns:
            np.array of sine wave values; successive calls return
            successive fragments of the sinusoidal.
        """
        #angle = self.phase + 2 * np.pi * self.frequency * np.arange(num_samples) * self.time_step
        # update phase
        #self.phase = angle[-1]
        #return self.amplitude * np.sin(angle)
        data = []
        i = 0
        while i < num_samples:
            remaining_samples = num_samples - i
            if self.ptr + remaining_samples > self.samples_in_a_cycle:
                remaining_samples = self.samples_in_a_cycle - self.ptr 

            data.extend(self.data[self.ptr:self.ptr+remaining_samples])
            i = i + remaining_samples
            self.ptr = self.ptr + remaining_samples
            if self.ptr >= self.samples_in_a_cycle:
                self.ptr = 0
        return np.array(data)


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    s = Signal(amplitude=2**10, frequency=441, sampling_rate=16000,phase=np.pi/4.0)

    data = np.array([],dtype='int16')

    for i in range(7):
        data = np.concatenate((data,s.sine(13)))

    fig, ax = plt.subplots()

    ax.plot(data)

    plt.show()

