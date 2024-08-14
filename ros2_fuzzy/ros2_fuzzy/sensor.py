import math
import pymunk


class Sensor():
    def __init__(self,ray_start, max_dist, ray_number, shape):
        
        self.ray_start = ray_start
        
        self.max_distance = max_dist
        self.ray_number = ray_number
        self.radian_angle = 70
        self.agent_shape = shape

    def raycast(self,ray_direction, space):
        ray_end = (self.ray_start[0] + ray_direction[0] * self.max_distance, self.ray_start[1] + ray_direction[1] * self.max_distance)
        info = space.segment_query(self.ray_start, ray_end, 0, pymunk.ShapeFilter())

        filtered_info = [collision for collision in info if collision.shape is not self.agent_shape]

        if filtered_info:
            return filtered_info
        else:
            return None

    def is_intersect(self,angle,space):
        #min_distance = float("inf")
        distances = []
        angle_step = math.radians(self.radian_angle) / (self.ray_number - 1)  # Angle between each ray
        #closest_intersection_point = None
        for i in range(self.ray_number):
            angle_offset = -math.radians(self.radian_angle/2) + i * angle_step  # Offset to cover a 50 degree
            direction = (math.cos(angle + angle_offset), math.sin(angle + angle_offset))
            results = self.raycast( direction,space)
            
            # Draw the rays and print the results
            if results:
                for info in results:
                    intersection_point = info.point
                    distance = math.hypot(intersection_point[0] - self.ray_start[0], intersection_point[1] - self.ray_start[1])
                    distances.append(distance)
                    # # If the distance is smaller, update the closest point and distance
                    # if distance < min_distance:
                    #     min_distance = distance
                    #     closest_intersection_point = intersection_point
                       
                    # #print(f"Ray hit a shape at {intersection_point} with direction {direction} and distance {distance}")

        # if closest_intersection_point:
        #     print(f"Closest intersection point: {closest_intersection_point} with distance {min_distance}")

        return distances