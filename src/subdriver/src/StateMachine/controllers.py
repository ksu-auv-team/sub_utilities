#! /usr/bin/env/python2
'''Controllers is a small collection of basic contol algorithms.
These algorithms are generalized for application in various parts
of the codebase.
'''

class PID:
  '''
  PID Controller implementation.
  Args:
    p (float): proportional constant coefficient
    i (float): integral constant coefficient
    I (float): integral value
    I_max (float): maximum integral value
    I_min (float): minimum integral value
    d (float): derivative constant coeffficient
    D (float): derivative value
    s (float): set point
  '''
  def __init__(self, p=2.0, i=0.0, I=0.0, I_max=250.0, I_min=-250.0, d=1.0, D=0, s=0):
    self.kp = p
    self.ki = i
    self.kd = d
    self.D = D
    self.I = I
    self.I_max = I_max
    self.I_min = I_min
    self.set_point = s
    self.error = 0.0

  def Update(self, current_value):
    '''Calculates controlled output for updated input value.
    Args:
      current_value (float): current value of the controlled variable.
    Returns:
      (float): controlled output to apply to variable.
    '''
    self.error = self.set_point - current_value
    P_val = self.kp * self.error
    D_val = self.kd * (self.error - self.D)
    self.D = self.error
    self.I += self.error
    if self.I > self.I_max:
      self.I = self.I_max
    if self.I < self.I_min:
      self.I = self.I_min
    I_val = self.I * self.ki

    return P_val + I_val + D_val

class PI:
  '''
  PI Controller implementation.
  Args:
    p (float): proportional constant coefficient
    i (float): integral constant coefficient
    I (float): integral value
    I_max (float): maximum integral value
    I_min (float): minimum integral value
    s (float): set point
  '''
  def __init__(self, p=2.0, i=0.0, I=0.0, I_max=250.0, I_min=-250.0, s=0):
    self.kp = p
    self.ki = i
    self.I = I
    self.I_max = I_max
    self.I_min = I_min
    self.set_point = s
    self.error = 0.0

  def Update(self, current_value):
    '''Calculates controlled output for updated input value.
    Args:
      current_value (float): current value of the controlled variable.
    Returns:
      (float): controlled output to apply to variable.
    '''
    self.error = self.set_point - current_value
    P_val = self.kp * self.error
    self.I += self.error
    if self.I > self.I_max:
      self.I = self.I_max
    if self.I < self.I_min:
      self.I = self.I_min
    I_val = self.I * self.ki

    return P_val + I_val

class PD:
  '''
  PD Controller implementation.
  Args:
    p (float): proportional constant coefficient
    d (float): derivative constant coeffficient
    D (float): derivative value
    s (float): set point
  '''
  def __init__(self, p=2.0, d=1.0, D=0, s=0):
    self.kp = p
    self.kd = d
    self.D = D
    self.set_point = s
    self.error = 0.0

  def Update(self, current_value):
    '''Calculates controlled output for updated input value.
    Args:
      current_value (float): current value of the controlled variable.
    Returns:
      (float): controlled output to apply to variable.
    '''
    self.error = self.set_point - current_value
    P_val = self.kp * self.error
    D_val = self.kd * (self.error - self.D)
    self.D = self.error

    return P_val + D_val



class P:
  '''
  P Controller implementation.
  Args:
    p (float): proportional constant coefficient
    s (float): set point
  '''
  def __init__(self, p=2.0, s=0):
    self.kp = p
    self.set_point = s
    self.error = 0.0

  def Update(self, current_value):
    '''Calculates controlled output for updated input value.
    Args:
      current_value (float): current value of the controlled variable.
    Returns:
      (float): controlled output to apply to variable.
    '''
    self.error = self.set_point - current_value
    P_val = self.kp * self.error

    return P_val