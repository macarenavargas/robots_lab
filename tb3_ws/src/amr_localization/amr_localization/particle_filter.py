import datetime
import math
import numpy as np
import os
import pytz
import random

from amr_localization.maps import Map
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        particle_count: int,
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
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._logger = logger
        self._particle_count: int = particle_count
        self._sensor_range_max: float = sensor_range_max
        self._sensor_range_min: float = sensor_range_min
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._simulation: bool = simulation
        self._iteration: int = 0
        self._num_rays = 8

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

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        # TODO: 3.10. Complete the missing function body with your code.
        localized: bool = False
        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))

        # extract the data
        x_p = self._particles[:, 0].astype(float)
        y_p = self._particles[:, 1].astype(float)
        th_p = self._particles[:, 2].astype(float)
        features = np.column_stack((x_p, y_p, np.sin(th_p), np.cos(th_p)))

        # create the Clustering object
        # eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
        db = DBSCAN(eps=0.2, min_samples=15).fit(features)
        # labels_: Cluster labels for each point in the dataset given to fit().
        # Noisy samples are given the label -1. Non-negative integers indicate cluster membership.
        labels = db.labels_

        # extract the unique labels of each cluster
        unique_labels = set(labels)

        # remove the noise label
        if -1 in unique_labels:
            unique_labels.remove(-1)

        n_clusters = len(unique_labels)
        self._logger.info(f"Number of clusters: {n_clusters}")

        # if we only have one cluster left -> localized
        if n_clusters == 1:
            # since we create a new dbscan object each iteration, when we only have one cluster left,
            # we know that its label assigned will be 0
            mask = labels == 0
            # count how many particles belong to the one cluster
            percentage_in_cluster = np.sum(mask) / self._particle_count

            # security filter to make sure that most of the particles are actually in the cluster
            if percentage_in_cluster > 0.7 and np.sum(mask) > 30:
                self._logger.info(f"Clustering criteria accomplished ")
                # the robot has been localized
                localized = True

                # calculate centroid with atan2
                x_hat = float(x_p[mask].mean())
                y_hat = float(y_p[mask].mean())
                th_hat = float(math.atan2(np.sin(th_p[mask]).mean(), np.cos(th_p[mask]).mean()))

                pose = (x_hat, y_hat, th_hat)

                # 50?
                self._particle_count = 50
            else:
                # means that some particles have been grouped by chance, but in reality most of the particles are diseprsed
                localized = False

        # elif n_clusters > 1:
        elif n_clusters > 1:
            # if we have multiple clusters
            localized = False
            # asign 100 particles for each cluster that exists
            particles_needed = n_clusters * 200  # 100

            # we put an upper bound limit so that we dont create more particles than self._initial_particle_count
            # we put an lower bound so that we habe at leas 200 particles during the whole process
            self._particle_count = min(self._initial_particle_count, max(200, particles_needed))

            # tests to see optimal methods  :
            # self._particle_count = self._initial_particle_count
            # self._particle_count = max(200, self._particle_count //2 )
            self._logger.info(f"Particles que hay ahora: {particles_needed}")

        else:
            # n_clusters == 0: the robot is lost
            localized = False
            self._particle_count = self._initial_particle_count

        return localized, pose

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].

        """
        self._iteration += 1

        # TODO: 3.5. Complete the function body with your code.

        # we update the pose and orentation of each particle
        for i in range(len(self._particles)):
            # add gaussian noise to v and w
            v_gauss = v + np.random.normal(0, self._sigma_v)
            w_gauss = w + np.random.normal(0, self._sigma_w)
            x_prev, y_prev, theta_prev = self._particles[i]
            # calulate x position
            x_new = x_prev + v_gauss * np.cos(theta_prev) * self._dt
            # calulate y position
            y_new = y_prev + v_gauss * np.sin(theta_prev) * self._dt
            # calulate theta and normalize so that the value is between [0,2pi]
            if self._simulation:
               theta_new = (theta_prev - w_gauss * self._dt) % (2 * math.pi)
            else:
                theta_new = (theta_prev - w_gauss * self._dt) % (2 * math.pi)

            # calculate if its outside the allowed boundaries
            intersection, _ = self._map.check_collision([(x_prev, y_prev), (x_new, y_new)], False)

            # if it update x and y to the first intersection point
            if intersection:
                x_new = intersection[0]
                y_new = intersection[1]
            # update the particle's pose
            self._particles[i] = (x_new, y_new, theta_new)

    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles.

        Args:
            measurements: Sensor measurements [m].

        """
        # TODO: 3.9. Complete the function body with your code (i.e., replace the pass statement).

        measurements = self._extract_robust_measurements(measurements)

        # calculate the weights for each particel (their probability)
        weights = np.array(
            [self._measurement_probability(measurements, p) for p in self._particles]
        )
        N = self._particle_count
        W = np.sum(weights)

        if W == 0:
           
            weights = np.ones(self._particle_count) / self._particle_count
        else:
          
            weights = weights / W

        # create an array with the cumulative sum
        weight_ruler = np.cumsum(weights)
        N = self._particle_count
        step = 1.0 / N
        # take the first sample starting point
        start = np.random.uniform(0, step)

        # generate the N points we are going to select out of the array
        selection_points = start + np.arange(N) * step

        # with digitize we asociate the selection points with their corresponding index in the weight ruler array
        new_idxs = np.digitize(selection_points, weight_ruler)
        # new_idxs = np.minimum(new_idxs, self._particle_count - 1) # ??? error númerico
        # we filter the array bu the selected idxs and update the particle list.
        self._particles = self._particles[new_idxs].copy()
    def _extract_robust_measurements(self, measurements: list[float], window_size: int = 3) -> np.ndarray:
        """Extracts and cleans LiDAR measurements depending on the environment.
        
        Applies a neighborhood search (window filter) for real-world noisy data, 
        and a simple cleanup for perfect simulation data.
        """
        num_total_measurements = len(measurements)
        idxs_real = np.linspace(0, num_total_measurements - 1, self._num_rays, dtype=int)
        
        raw_measurements = np.array(measurements)
        raw_measurements[np.isinf(raw_measurements)] = np.nan

        robust_measurements = []

        if self._simulation:
            # --- MODO SIMULACIÓN ---
            # El simulador es perfecto. Si devuelve un NaN (infinito), significa 
            # que el rayo no chocó con nada. Asignamos el rango máximo.
            for idx in idxs_real:
                val = raw_measurements[idx]
                if np.isnan(val):
                    robust_measurements.append(self._sensor_range_max)
                else:
                    robust_measurements.append(val)
                    
        else:
            # --- MODO ROBOT REAL ---
            # Filtramos los valores basura (el láser LDS-01 falla por debajo de 0.16m)
            raw_measurements[raw_measurements < self._sensor_range_min] = np.nan
            
            for idx in idxs_real:
                val = raw_measurements[idx]
                
                # Si el rayo principal es válido, genial
                if not np.isnan(val):
                    robust_measurements.append(val)
                else:
                    # Búsqueda de vecindad (Window Search)
                    found_valid = False
                    for offset in range(1, window_size + 1):
                        # Miramos a la derecha
                        idx_r = (idx + offset) % num_total_measurements
                        if not np.isnan(raw_measurements[idx_r]):
                            robust_measurements.append(raw_measurements[idx_r])
                            found_valid = True
                            break
                            
                        # Miramos a la izquierda
                        idx_l = (idx - offset) % num_total_measurements
                        if not np.isnan(raw_measurements[idx_l]):
                            robust_measurements.append(raw_measurements[idx_l])
                            found_valid = True
                            break
                    
                    # Si toda la vecindad es NaN, devolvemos NaN. 
                    # Esto indicará a la probabilidad que ignore este rayo.
                    if not found_valid:
                        robust_measurements.append(float('nan'))

        return np.array(robust_measurements)
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
        particles = np.empty((particle_count, 3), dtype=object)
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

    def _sense(self, pose: tuple[float, float, float]) -> list[float]:
        """Obtains the predicted measurement of every LiDAR ray given the robot's pose.

        Args:
            pose: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; nan if a sensor is out of range.

        """

        z_hat: list[float] = []

        # TODO: 3.6. Complete the missing function body with your code.

        # assuming that the lidar array has 360 values, we extract 8 that are equally spaced out
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
        particle_measurements = self._sense(particle)
        # clean the nans of the particle's measurement, explain max
        particle_measurements = np.nan_to_num(xºparticle_measurements, nan=self._sensor_range_max)

        # claculate the likelihood of the particle with the robot
        for i in range(self._num_rays):
            if not np.isnan(measurements[i]):
                probability *= self._gaussian(particle_measurements[i], self._sigma_z, measurements[i])

        return probability
