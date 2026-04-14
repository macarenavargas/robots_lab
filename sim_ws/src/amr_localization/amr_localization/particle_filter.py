import datetime
import math
import numpy as np
import os
import pytz
import random
import time 

from amr_localization.maps import Map
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN


# import the c++ module
import sys
sys.path.append("/workspaces/robots_lab/sim_ws/install/amr_cpp/lib")

import cpp_module


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        particle_count: int,
        # probar a aumentar, jugar con estos valores
        sigma_v: float = 0.05,  # initial value : 0.05
        sigma_w: float = 0.1,  # initial value : 0.1
        sigma_z: float = 0.2,  # initial value : 0.2
        sensor_range_max: float = 8.0,
        sensor_range_min: float = 0.16,
        global_localization: bool = True,
        initial_pose: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        initial_pose_sigma: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        logger=None,
        simulation: bool = False,
    ):
        """Particle filter class initializer.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].
            sensor_range_max: Maximum sensor measurement range [m].
            sensor_range_min: Minimum sensor measurement range [m].
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].
            logger: Logger object to output messages with different severity levels.
            simulation: True if running in simulation, False if running on the real robot.

        """
        # particle_count = 500SSSS
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._logger = logger
        self._particle_count: int = particle_count  # particle_count
        self._sensor_range_max: float = sensor_range_max
        self._sensor_range_min: float = sensor_range_min
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._simulation: bool = simulation
        self._iteration: int = 0
        self._num_rays = 8  # 8
        # self._localized = False

        self._map = Map(
            map_path,
            sensor_range_max,
            compiled_intersect=True,
            use_regions=False,
            safety_distance=0.08,
        )
        self._particles = self._init_particles(
            particle_count, global_localization, initial_pose, initial_pose_sigma
        )
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

        # adding feature -> use C++ or not 
        self._use_cpp = True  

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        # TODO: 3.10. Complete the missing function body with your code.

        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))

        # STEP 1: extract the data
        x_p = self._particles[:, 0].astype(float)
        y_p = self._particles[:, 1].astype(float)
        th_p = self._particles[:, 2].astype(float)
        features = np.column_stack((x_p, y_p, np.sin(th_p), np.cos(th_p)))

        # STEP 2: create the Clustering object
        # eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
        if self._simulation:
            db = DBSCAN(eps=0.2, min_samples=15).fit(features)
        else:
            db = DBSCAN(eps=0.1, min_samples=15).fit(features)

        # STEP 3: Extract the labels of the current clusters
        labels = db.labels_

        # extract the unique labels of each cluster
        unique_labels = set(labels)

        # remove the noise label
        if -1 in unique_labels:
            unique_labels.remove(-1)

        n_clusters = len(unique_labels)
        localized = False
        # STEP 4: compute actions based on the number of clusters
        if n_clusters == 1:
            # since we create a new dbscan object each iteration, when we only have one cluster left,
            # we know that its label assigned will be 0
            mask = labels == 0
            # count how many particles belong to the one cluster
            percentage_in_cluster = np.sum(mask) / self._particle_count

            # security filter to make sure that most of the particles are actually in the cluster
            if percentage_in_cluster > 0.7 and np.sum(mask) > 30:
                x_hat = float(x_p[mask].mean())
                y_hat = float(y_p[mask].mean())
                th_hat = float(math.atan2(np.sin(th_p[mask]).mean(), np.cos(th_p[mask]).mean()))
                pose = (x_hat, y_hat, th_hat)
                self._particle_count = 50

                localized = True
            else:
                # means that some particles have been grouped by chance, but in reality most of the particles are diseprsed
                localized = False

        elif n_clusters > 1:
            localized = False
            particles_needed = n_clusters * 100
            self._particle_count = min(self._initial_particle_count, max(200, particles_needed))
            if self._logger:
                self._logger.info(
                    f"MULTIPLE HIPOTHESIS: There are {n_clusters} posible locations (clusters). Active particles: {self._particle_count}."
                )

        else:
            localized = False
            self._particle_count = self._initial_particle_count
            if self._logger:
                self._logger.warning(
                    "LOCALIZATION LOST: The particles have dispersed, restarting global localization."
                )

        return localized, pose

    def check_likelihood(
        self,
        pose: tuple[float, float, float],
        measurements: list[float],
        expected_likelihood: float = 1.5,
    ) -> bool:
        """
        Detecta falsa convergencia o secuestro midiendo la discrepancia
        de la verosimilitud media, tal y como indica el enunciado.

        Args:
            pose: Pose estimada a evaluar.
            measurements: Lecturas reales del LiDAR.
            expected_likelihood: Verosimilitud media que se obtiene cuando el robot converge bien (A calibrar empíricamente).
            tolerance: Porcentaje de caída permitido antes de considerar que hay "demasiada discrepancia" (ej. 0.5 = 50%).
        """

        if math.isinf(pose[0]):
            return False

        # 1. Extraemos las medidas reales
        z_real = self._extract_robust_measurements(measurements)

        # 2. Simulamos las medidas que DEBERÍA tener esa pose según el mapa
        z_simulated = self._sense(pose)
        z_simulated = np.nan_to_num(z_simulated, nan=self._sensor_range_min)

        # 3. Calculamos la VEROSIMILITUD MEDIA de las medidas de los sensores
        total_likelihood = 0.0
        for i in range(self._num_rays):
            total_likelihood += self._gaussian(z_simulated[i], self._sigma_z, z_real[i])

        average_likelihood = total_likelihood / self._num_rays

        if self._logger:
            self._logger.info(
                f"Verosimilitud media: {average_likelihood:.2f} | Referencia: {expected_likelihood:.2f}"
            )

        # 4. Comprobamos 
        if average_likelihood < (expected_likelihood):
            if self._logger:
                self._logger.error(
                    f" ROBOT ESTÁ PERDIDO"
                    f"({average_likelihood:.2f} es inferior a {expected_likelihood:.2f}). "
                    f"La solución consiste en reiniciar el filtro..."
                )

            # 5. La solución: reiniciar el filtro por completo
            self._particles = self._init_particles(
                self._initial_particle_count,
                global_localization=True,
                initial_pose=(0.0, 0.0, 0.0),
                initial_pose_sigma=(0.0, 0.0, 0.0),
            )
            self._particle_count = self._initial_particle_count

            return False

        # La pose es válida
        return True

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].

        """
        self._iteration += 1
        start = time.time() 
        # TODO: 3.5. Complete the function body with your code.

        n_particles = len(self._particles)

        # Calcualte gaussian noise for all particles
        v_gauss = v + np.random.normal(0, self._sigma_v, n_particles)
        w_gauss = w + np.random.normal(0, self._sigma_w, n_particles)

        # self._logger.info(f"move the particle {v,w}")

        # extract value arrays
        x_prev = self._particles[:, 0]
        y_prev = self._particles[:, 1]
        theta_prev = self._particles[:, 2]

        # calulate y position
        x_new = x_prev + v_gauss * np.cos(theta_prev) * self._dt
        # calulate y position
        y_new = y_prev + v_gauss * np.sin(theta_prev) * self._dt
        # calulate theta and normalize so that the value is between [0,2pi]
        if self._simulation:
            theta_new = (theta_prev - w_gauss * self._dt) % (2 * math.pi)
        else:
            theta_new = (theta_prev + w_gauss * self._dt) % (2 * math.pi)

        # we update the pose and orentation of each particle
        for i in range(n_particles):
            intersection, _ = self._map.check_collision(
                [(x_prev[i], y_prev[i]), (x_new[i], y_new[i])], False
            )
            # if it update x and y to the first intersection point
            if intersection:
                x_new[i] = intersection[0]
                y_new[i] = intersection[1]

        self._particles[:, 0] = x_new
        self._particles[:, 1] = y_new
        self._particles[:, 2] = theta_new


        print("MOVING TIME :", time.time() - start)

    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles.

        Dispatches to the C++ or Python implementation depending on self._use_cpp.

        Args:
            measurements: Sensor measurements [m].

        """
        if self._use_cpp:
            self._resample_cpp(measurements)
        else:
            self._resample_python(measurements)

    def _resample_python(self, measurements: list[float]) -> None:
        """Python implementation of the resample step.

        Args:
            measurements: Sensor measurements [m].

        """
        # STEP 1: extract the 8 lidar values and clean them.
        z_real = self._extract_robust_measurements(measurements)

        # STEP 2: calcualte the weights of each particle (how important they are)
        weights = np.zeros(len(self._particles))
        for i, particle in enumerate(self._particles):
            weights[i] = self._measurement_probability(z_real, particle)

        # STEP 4: Normalize the weights.
        weight_sum = np.sum(weights)
        if weight_sum > 0:
            weights /= weight_sum
        else:
            # safety preacaution: If every particle has 0 probability -> asign uniform weights
            weights = np.ones(len(self._particles)) / len(self._particles)

        # STEP 5: apply sistematic resampling
        N = self._particle_count  # get current particle count ( updated by dbscan )

        # generate 1 random number between 0 and 1/N and create and array of pointers
        positions = (np.random.random() + np.arange(N)) / N

        # create cumulative array of weights
        cumulative_sum = np.cumsum(weights)

        # find the indexes where the pointers fall over the cumulative sum
        indices = np.digitize(positions, cumulative_sum)

        # STEP 6: update the aprticle array with the surviving particles.
        self._particles = self._particles[indices]

    def _resample_cpp(self, measurements: list[float]) -> None:
        """C++ implementation of the resample step.

        Calls cpp_module.resample(), which runs steps 1-6 entirely in C++:
        extract measurements → compute weights (sense + Gaussian) →
        normalize → systematic resampling → return new particles.

        Args:
            measurements: Sensor measurements [m].

        """
        new_particles, average_likelihood = cpp_module.resample(
            self._particles.tolist(),
            list(measurements),
            self._map._map_segments,
            self._num_rays,
            self._sensor_range_max,
            self._sensor_range_min,
            self._sigma_z,
            self._particle_count,
        )

        self._particles = np.array(new_particles)
        self.average_likelihood = average_likelihood

        if self._logger:
            self._logger.warning(f"AVG WEIGHT {self.average_likelihood} ")



    def _extract_robust_measurements(
        self, measurements: list[float], window_size: int = 3
    ) -> np.ndarray:
        """Extracts and cleans LiDAR measurements.

        Extrae exactamente 'self._num_rays' y sustituye los valores
        inválidos (NaN, Inf) por el rango mínimo del sensor.
        """
        n_real = len(measurements)
        # STEP 1: safety precaution in case the lidar doesnt send anything
        if n_real == 0:
            return np.full(self._num_rays, self._sensor_range_min, dtype=float)

        # STEP 2: calculate the 8 lidar rays
        idxs = np.linspace(0, n_real - 1, self._num_rays, dtype=int)

        # STEP 3: initializy empty array for extracted rays
        extracted = np.empty(self._num_rays, dtype=float)

        # STEP 4: extract each ray and substitute for the minimum
        for i, idx in enumerate(idxs):
            val = measurements[idx]
            if math.isnan(val) or math.isinf(val) or val <= 0.0:
                extracted[i] = self._sensor_range_min
            else:
                extracted[i] = val

        return extracted

    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(
                self._particles[:, 0],
                self._particles[:, 1],
                dx,
                dy,
                color="b",
                scale=15,
                scale_units="inches",
            )
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], "bo", markersize=1)

        return axes

    def show(
        self,
        title: str = "",
        orientation: bool = True,
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)

        axes.set_title(title + " (Iteration #" + str(self._iteration) + ")")
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", save_dir, self._timestamp)
            )

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = str(self._iteration).zfill(4) + " " + title.lower() + ".png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _init_particles(
        self,
        particle_count: int,
        global_localization: bool,
        initial_pose: tuple[float, float, float],
        initial_pose_sigma: tuple[float, float, float],
    ) -> np.ndarray:
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3*pi/2].

        Args:
            particle_count: Number of particles.
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

        Returns: A NumPy array of tuples (x, y, theta) [m, m, rad].

        """
        self._logger.warn(f"particle count init {particle_count}")
        particles = np.empty((particle_count, 3), dtype=float)
        valid_orientations = [0, math.pi / 2, math.pi, math.pi * 3 / 2]
        x_min, y_min, x_max, y_max = self._map.bounds()

        # TODO: 3.4. Complete the missing function body with your code.

        num_particled_created = 0

        while num_particled_created < particle_count:
            if global_localization:
                # every point of the map is equally probable.
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
                theta = random.choice(valid_orientations)

            else:  # we make a gaussian distribution centered on the estimated position of the robot
                # and with the indicated desviation
                x = np.random.normal(initial_pose[0], initial_pose_sigma[0])
                y = np.random.normal(initial_pose[1], initial_pose_sigma[1])
                theta = np.random.normal(initial_pose[2], initial_pose_sigma[2])

            if self._map.contains((x, y)):
                particles[num_particled_created, 0] = x
                particles[num_particled_created, 1] = y
                particles[num_particled_created, 2] = theta
                num_particled_created += 1

        return particles

    # this is the _sense function from before. 
    # if C++ doesnt work, use this as _sense. 
    def _sense_python(self, pose: tuple[float, float, float]) -> list[float]:
        """Obtains the predicted measurement of every LiDAR ray given the robot's pose.

        Args:
            pose: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; nan if a sensor is out of range.

        """
    
        z_hat: list[float] = []

        # TODO: 3.6. Complete the missing function body with your code.

        # assuming that the lidar array has 241 values, we extract 8 that are equally spaced out
        idxs = np.linspace(0, 239, self._num_rays, dtype=int)

        # extract the lidar rays in segment format
        selected_rays = self._lidar_rays(pose, idxs)

        for ray in selected_rays:
            # calcualate the distance of each ray from the robot to any obstacle if there is any
            _, distance = self._map.check_collision(ray, True)
            if distance is None:
                z_hat.append(float("nan"))
            else:
                z_hat.append(distance)
       
        return z_hat
    


    # sense that implements the c++ version
    def _sense_cpp(self, pose):
        return cpp_module.sense(
            pose.tolist(),
            self._map._map_segments,
            self._num_rays,
            self._sensor_range_max
        )

    def _sense(self, pose:tuple[float, float, float])-> list[float]: 
        
        if self._use_cpp: # if we want to use C++
            return self._sense_cpp(pose)
        else: # if we want to use Python 
            return self._sense_python(pose)

  
  
    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian value.

        """
        # TODO: 3.7. Complete the function body (i.e., replace the code below).
        # error between robots measurment and particle measurment
        error = x - mu
        # Si el error es 0, el resultado es el máximo posible
        # gauss formula
        num = math.exp(-(error**2) / (2 * sigma**2))
        den = sigma * math.sqrt(2 * math.pi)
        return num / den

    def _lidar_rays(
        self, pose: tuple[float, float, float], indices: tuple[float], degree_increment: float = 1.5
    ) -> list[list[tuple[float, float]]]:
        """Determines the simulated LiDAR ray segments for a given robot pose.

        Args:
            pose: Robot pose (x, y, theta) in [m] and [rad].
            indices: Rays of interest in counterclockwise order (0 for to the forward-facing ray).
            degree_increment: Angle difference of the sensor between contiguous rays [degrees].

        Returns: Ray segments. Format:
                 [[(x0_start, y0_start), (x0_end, y0_end)],
                  [(x1_start, y1_start), (x1_end, y1_end)],
                  ...]

        """
        x, y, theta = pose

        # Convert the sensor origin to world coordinates
        x_start = x - 0.035 * math.cos(theta)
        y_start = y - 0.035 * math.sin(theta)

        rays = []

        for index in indices:
            ray_angle = math.radians(degree_increment * index)
            x_end = x_start + self._sensor_range_max * math.cos(theta + ray_angle)
            y_end = y_start + self._sensor_range_max * math.sin(theta + ray_angle)
            rays.append([(x_start, y_start), (x_end, y_end)])

        return rays

    def _measurement_probability(
        self, measurements: list[float], particle: tuple[float, float, float]
    ) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with
        the minimum sensor range to perform the computation because the environment is smaller
        than the maximum range.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns:
            float: Probability.

        """
        probability = 1.0

        # TODO: 3.8. Complete the missing function body with your code.

        # take the particles ray measurements
        start = time.time()
        particle_measurements = self._sense(particle)
        print("SENSING TIME :", time.time() - start)

        particle_measurements = np.nan_to_num(particle_measurements, nan=self._sensor_range_min)

        # calculate the likelihood of the particle with the robot
        for i in range(self._num_rays):
            probability *= self._gaussian(particle_measurements[i], self._sigma_z, measurements[i])

        return probability
