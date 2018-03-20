# PID Controller project
-------
## Files included:
1. `reflection.md` is this file. It is a summary of how the PID controller is implemented, tuned, and possible improvement for future.
2. `Recording.mov` is the recording of car driving for a circle using the current PID parameters. Note that parameters were tuned without the screen recorder. The PID controller behaved slightly worse in the recording due to dropped frame rate.
3. `src/PID.cpp` is the implementation of the PID controller.
4. `src/main.cpp` is the main function that calls the PID controller. The throttle is set to a constant of 0.5 here, giving a maximum driving speed around 57 mph. It also loads the PID parameters from file `config.in` and saves the history into file `history.log`.
5. `config.in` is the file containing the tuned PID parameters `p = 0.1, i = 0.0015, d = 4`. Loading parameters from a file makes it easier to tune parameters without re-compiling the project.
6. `history.log` contains the history of the PID error during this drive. It is mainly used for debug and parameter tuning.

## PID controller
The PID controller is a standard method to implement a feedback loop, where
```
feedback = -total_error
= -(p_error + i_error + d_error)
= -(Kp * cte + Ki * Integral(cte) + Kd * Derivative(cte))
```

The Integral function can be simply implemented as a sum of all previous cte values. In this example, because the loop is counterclockwise, the integral part is expected to be positive most of the time.

The Derivative part was first implemented as `cte[t] - cte[t-1]`. However, that caused `d_error` to be very noise with lots of large spikes. Thus I used a smoothed version of derivative by defining it as the slope of the last **5** observations of cte values. Further increasing the width of the smoothing table can help reduce the noise but would cause a time delay in `d_error`.

## Parameter tuning
PID controller is very easy to implement but the parameter tuning took a long time.

In this project, I tuned the parameters through manual tuning with the throttle set to 0.5.

1. The first step is to estimate reasonable initial values for `Kp, Ki, Kd`. Because the feedback (steering) has to be between [-1, 1], ideally all 3 terms `p_error, i_error, d_error` will be between [-0.2, 0.2] so the `total_error` would not be dominated by only one term. In the simulator, the initial (and expected typical) `cte` value is around 0.8. I set the initial values as `Kp = 0.2, Ki = 0, Kd = 100`. The rough magnitude of `Ki` and `Kd` values will be estimated during tuning.
2. Using the initial values, the car was able to drive but the direction oscillates very fast, showing that 'Kd' was too large. It tends to stay on the left or right side of the road, show that 'Ki' is needed. So I set the parameters to `Kp = 0.2, Ki = 0.001, Kd = 10`. This `Ki` value is assuming that the car will stabilize to the center position in around 100 observations.
3. The above parameter set was already able to make the car finish a loop with a fast changing direction and just a few occasions of car driving out of the road. Then I tried to tune each parameter one at a time using step sizes `[0.05, 0.0005, 2]` for each parameter. `Kp = 0.1, Ki = 0.0015, Kd = 4` was the best combination I could find with those step sizes. It should be possible to fine-tune those parameters with smaller steps.
4. The "best" parameter found in the last step cannot be universally used. In the final test and recording, the best PID parameters seems to depend heavily on the time/distance between observations because of the way `i_error` and `d_error` are calculated. The best parameters found at `speed = 50 mph` would not work well at a sufficiently higher or lower speed. This can be confirmed by the fact that the car always performs worse when starting from `speed = 0` and drives smoothly when passing the same spot at the normal speed. Also, during the screen recording, the dropped frame rate also caused the PID controller to take longer the stabilize with the same parameters.

## Reflection on the PID parameters
There is no clear physical meaning for PID parameters. Here is my intuition on the effect of each parameter.
* `Kp` is how sensitive the controller is to the error at the **current observation**. A reasonable `Kp` helps the system to respond quickly, but `Kp` value being too large will make the system over-react to a small deviation to the central position.
* `Ki` is how sensitive the controller is to the **accumulated** error in **all** previous observations. The `i_error` does not change too much from the previous observation, and having this term helps the car to keep similar steering angle over several observations. For a working controller, the actual position is around the central position most of the time, and `i_error` mainly reflects the error in the last few observations. Thus it works similar as the `p_error` term but in a smoothed way.
* `Kd` term effectively **predicts** the future position by looking at the derivative in the last few observations. It has two effects. First, it prevents the car from overshooting when stabilizing from an off-center position to the central position. Second, when there is a sharp turn, `Kp` and `Ki` terms cannot respond quick enough. The `Kd` term _predicts_ the car position in the next few observations, and tells the car to turn before been too far from the central position. The `Kd` term also has a side-effect of causing the car oscillates spontaneously when `Kd` is too large, because the derivative term is very sensitive to any small change/error in car position.

## About the throttle (speed)
Following the feedback from the review, there is no need to use a constant throttle. In fact, a normal human driver tends to slow down when the steering angle is large and increase the throttle only on straighter roads. Therefore, a steering-angle-dependent throttle function `throttle = 1 - fabs(steering_angle)` is used instead of a constant. The parameters are therefore updated to be `Kp = 0.08, Ki = 0.0018, Kd = 6`. This allows the car to reach maximum speed near 100 mph and average speed over 80 mph.
