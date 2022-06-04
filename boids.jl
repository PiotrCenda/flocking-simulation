# Boid struct 
mutable struct Boid
    id
    position
    velocity
    acceleration
end

# Parameters
max_force = 3.5
max_speed = 23.
edge_radius = 250
edgeValue = 1.2
max_edge_force = max_force * 1.8
max_edge_speed = max_speed * 1.8

# limit the magnitude of a 2-d Array to lim
function set_limit!(v::Array, lim::Float64)
    n = sqrt(sum(v.^2))
    f = min(n, lim)/n
    return v .* f
end

# set the magnitude of a 2-d Array to mag
function set_magnitude!(v::Array, mag::Float64)
    n = sqrt(sum(v.^2))
    f = max(n, mag)/n
    return v .* f
end

# flap updates the acceleration of each boid
function flap!(boid::Boid, flock::Vector{Boid}, alignValue, separationValue, cohesionValue, axis_limit, align_perception_radius, separate_perception_radius, cohese_perception_radius)
    boid.acceleration += align!(boid, flock, align_perception_radius) .* alignValue
    boid.acceleration += seperate!(boid, flock, separate_perception_radius) .* separationValue
    boid.acceleration += cohese!(boid, flock, cohese_perception_radius) .* cohesionValue
    boid.acceleration += egde!(boid, axis_limit) .* edgeValue
    boid.acceleration += egde!(boid, axis_limit+(edge_radius*0.95)) .* (edgeValue*3)
end

## fly updates the position and velocity of each boid
# each boid flaps ones per fly update
function fly!(boid::Boid, flock::Vector{Boid}, alignValue, separationValue, cohesionValue, axis_limit, align_perception_radius, separate_perception_radius, cohese_perception_radius)
    # max_speed = 5.
    flap!(boid, flock, alignValue, separationValue, cohesionValue, axis_limit, align_perception_radius, separate_perception_radius, cohese_perception_radius)
    boid.position += boid.velocity
    boid.velocity += boid.acceleration
    boid.velocity = set_limit!(boid.velocity, max_speed)
    boid.acceleration .*= 0
end

# align Boid to its flock 
function align!(b::Boid, flock::Vector{Boid}, align_perception_radius)
    # max_force = 2.
    # max_speed = 5.
    steering = zeros(size(b.velocity))
    total = 0

    for boid in flock
        dist = b.position - boid.position
        if b.id != boid.id && norm(dist) < align_perception_radius
            steering += boid.velocity
            total +=1
        end
    end

    if total > 0
        steering ./= total
        steering = set_magnitude!(steering, max_speed)
        steering -= b.velocity
        steering = set_limit!(steering, max_force)
    end
    return steering       
end

# seperate each flock from its flock s.t. they dont collide
function seperate!(b::Boid, flock::Vector{Boid}, separate_perception_radius)
    # max_force = 2.
    # max_speed = 5.
    steering = zeros(size(b.velocity))
    total = 0

    for boid in flock
        dist = b.position - boid.position
        if b.id != boid.id && norm(dist) < separate_perception_radius
            diff = dist / (norm(dist)^2)
            steering += diff
            total +=1
        end
    end    

    if total > 0
        steering ./= total
        steering = set_magnitude!(steering, max_speed)
        steering -= b.velocity
        steering = set_limit!(steering, max_force)
    end
    return steering       
end


function egde!(b::Boid, axis_limit)
    steering = zeros(size(b.velocity))
    total = 0

    # custom edge separation
    if (b.position[1] - axis_limit) < edge_radius
        plane_dist = [b.position[1] - axis_limit, 0, 0]
        if norm(plane_dist) < edge_radius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff
            total +=1
        end
    end
    if (b.position[1] + axis_limit) < edge_radius
        plane_dist = [b.position[1] + axis_limit, 0, 0]
        if norm(plane_dist) < edge_radius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff
            total +=1
        end
    end

    if (b.position[2] - axis_limit) < edge_radius
        plane_dist = [0, b.position[2] - axis_limit, 0]
        if norm(plane_dist) < edge_radius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff
            total +=1
        end
    end
    if (b.position[2] + axis_limit) < edge_radius
        plane_dist = [0, b.position[2] + axis_limit, 0]
        if norm(plane_dist) < edge_radius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff
            total +=1
        end
    end

    if (b.position[3] - axis_limit) < edge_radius
        plane_dist = [0, 0, b.position[3] - axis_limit]
        if norm(plane_dist) < edge_radius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff
            total +=1
        end
    end
    if (b.position[3] + axis_limit) < edge_radius
        plane_dist = [0, 0, b.position[3] + axis_limit]
        if norm(plane_dist) < edge_radius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff
            total +=1
        end
    end
    

    if total > 0
        steering ./= total
        steering = set_magnitude!(steering, max_edge_speed)
        steering -= b.velocity
        steering = set_limit!(steering, max_edge_force)
    end
    return steering       
end

# follow the flocks average position 
function cohese!(b::Boid, flock::Vector{Boid}, cohese_perception_radius)
    # max_force = 2.
    # max_speed = 5.
    steering = zeros(size(b.velocity))
    total = 0

    for boid in flock
        dist = b.position - boid.position
        if b.id != boid.id && norm(dist) < cohese_perception_radius
            steering += boid.position
            total +=1
        end
    end

    if total > 0
        steering ./= total
        steering -= b.position
        steering = set_magnitude!(steering, max_speed)
        steering -= b.velocity
        steering = set_limit!(steering, max_force)
    end
    return steering       
end
