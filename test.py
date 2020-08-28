from fitparse import FitFile
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy import signal
from scipy.optimize import minimize
from math import sin, cos, sqrt, atan, pi
from itertools import tee


def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)


def speed_increment(initial_speed, power, rider_mass, bike_mass, grade, delta_distance,
                    wind_speed, wind_direction, f1, f2):
    grade_theta = atan(grade/100.0)
    v_ave = initial_speed
    final_speed = 0
    delta_time = 0
    converged = False
    count = 0
    while not converged and count < 100:
        effective_power = power * (1.0 - f2)
        mg = (rider_mass + bike_mass) * 9.81
        acceleration = (effective_power / v_ave - mg * sin(grade_theta) - f1 * v_ave**2.0) / (rider_mass + bike_mass)
        v_ave_old = v_ave
        final_speed = sqrt(initial_speed**2.0 + 2.0 * acceleration * delta_distance)
        v_ave = (initial_speed + final_speed) / 2.0
        delta_time = delta_distance / v_ave
        error = abs(v_ave - v_ave_old)
        # print('Count, Final Speed = ', count, final_speed)
        count += 1
        if error <= 1.0e-9:
            converged = True
    if not converged:
        print("Bad hoodoo: ", power, rider_mass, bike_mass, grade, delta_distance, f1, f2, initial_speed, final_speed, delta_time)
    return final_speed, delta_time


def get_model_error(guess_vector, rider, bike, time, distance, grade, elevation, power, speed):
    # history = zip(time, distance, grade, power, pairwise(speed))
    history = zip(pairwise(time), pairwise(distance), grade, pairwise(elevation), power, pairwise(speed))
    print('Guess: ', guess_vector)
    speed_error = 0
    time_error = 0
    f_1, f_2 = guess_vector
    wind_speed = wind_direction = 0
    for entry in history:
        # print(entry)
        try:
            t_0, t_1 = entry[0]
            d_0, d_1 = entry[1]
            grade = entry[2]
            e_0, e_1 = entry[3]
            power = entry[4]
            v_0, v_1 = entry[5]
            delta_time = t_1 - t_0
            delta_dist = d_1 - d_0
            try:
                new_grade = 100.0 * (e_1 - e_0) / (d_1 - d_0)
            except ZeroDivisionError:
                new_grade = grade
            if power > 1.0 and delta_dist > 0:
                v_1_guess, dt_guess = speed_increment(v_0, power, rider, bike, grade, delta_dist,
                                wind_speed, wind_direction, f_1, f_2)
                speed_error += (v_1_guess - v_1)**2.0
                time_error += (dt_guess - delta_time)**2.0
        except TypeError:
            # print(entry)
            pass
        except ValueError:
            # print(entry)
            pass
    print('Speed, Time Error = ', speed_error, time_error)
    error = speed_error + time_error
    return error


fitfile = FitFile('Crawford_Co.fit')
# fitfile = FitFile('tdf_stage5.fit')
rider_mass = 65  # kg
bike_mass = 8.2 # kg
skipped = 0
distance = []
time = []
speed = []
power = []
grade = []
elevation = []
for record in fitfile.get_messages('record'):
    values = record.get_values()
    # print(values)
    try:
        this_distance = values['distance']
        this_time = values['timestamp'].timestamp()
        this_speed = values['enhanced_speed']
        this_elevation = values['enhanced_altitude']
        this_grade = values['grade']
        this_power = values['power']
        distance.append(this_distance)
        time.append(this_time)
        speed.append(this_speed)
        elevation.append(this_elevation)
        power.append(this_power)
        grade.append(this_grade)
    except KeyError:
        skipped += 1
        # print(values)
#
print('Total records: ' + str(len(distance)) + ' Skipped: ' + str(skipped))
# # smooth_elevation = signal.savgol_filter(elevation, 5, 3)
# # tck = interpolate.splrep(time, distance)
# # speed_2 = interpolate.splev(time, tck, der=1)
#
# plt.plot(distance, speed)
# # plt.plot(distance[2000:3500], speed_2[2000:3500])
# plt.show()
w0 = [0.2, 0.05]
bounds = ((0, 1.0), (0, 0.2))
args = (rider_mass, bike_mass, time, distance, grade, elevation, power, speed)
result = minimize(get_model_error, w0, args, bounds=bounds, method='Nelder-Mead')
print(result)
f1, f2 = result.x
for entry in zip(pairwise(time), pairwise(distance), grade, pairwise(elevation), power, pairwise(speed)):
    try:
        t_0, t_1 = entry[0]
        d_0, d_1 = entry[1]
        grade = entry[2]
        e_0, e_1 = entry[3]
        power = entry[4]
        v_0, v_1 = entry[5]
        delta_time = t_1 - t_0
        delta_dist = d_1 - d_0
        try:
            new_grade = 100.0 * (e_1 - e_0) / (d_1 - d_0)
        except ZeroDivisionError:
            new_grade = grade
        if power > 1.0 and delta_dist > 0:
            v_1_guess, dt_guess = speed_increment(v_0, power, rider_mass, bike_mass, new_grade, delta_dist,
                                                  0.0, 0.0, f1, f2)
            print(d_0, power, new_grade, grade, v_0, v_1, v_1_guess, delta_time, dt_guess)
    except:
        print('Bad Result!', entry)