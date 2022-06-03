# Boid struct 
mutable struct Boid
    id
    position
    velocity
    acceleration
end

# Parameters
edge_radius = 150
perception_radius_multiplier = 2.1
max_force = 5.
max_speed = 25.
egde_separation_force = 100000.

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
function flap!(boid::Boid, flock::Vector{Boid}, alignValue, separationValue, cohesionValue, axis_limit)
    boid.acceleration += align!(boid, flock) .* alignValue
    boid.acceleration += seperate!(boid, flock, axis_limit) .* separationValue
    boid.acceleration += cohese!(boid, flock) .* cohesionValue
end

## fly updates the position and velocity of each boid
# each boid flaps ones per fly update
function fly!(boid::Boid, flock::Vector{Boid}, alignValue, separationValue, cohesionValue, axis_limit)
    # max_speed = 5.
    flap!(boid, flock, alignValue, separationValue, cohesionValue, axis_limit)
    boid.position += boid.velocity
    boid.velocity += boid.acceleration
    boid.velocity = set_limit!(boid.velocity, max_speed)
    boid.acceleration .*= 0
end

# align Boid to its flock 
function align!(b::Boid, flock::Vector{Boid})
    # max_force = 2.
    # max_speed = 5.
    perceptionRadius = 50 * perception_radius_multiplier
    steering = zeros(size(b.velocity))
    total = 0

    for boid in flock
        dist = b.position - boid.position
        if b.id != boid.id && norm(dist) < perceptionRadius
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
function seperate!(b::Boid, flock::Vector{Boid}, axis_limit)
    # max_force = 2.
    # max_speed = 5.
    perceptionRadius = 50 * perception_radius_multiplier
    steering = zeros(size(b.velocity))
    total = 0

    for boid in flock
        dist = b.position - boid.position
        if b.id != boid.id && norm(dist) < perceptionRadius
            diff = dist / (norm(dist)^2)
            steering += diff
            total +=1
        end
    end

    # custom edge separation
    if (b.position[1] - axis_limit) < edge_radius
        plane_dist = [b.position[1] - axis_limit, 0, 0]
        if norm(plane_dist) < perceptionRadius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff.*egde_separation_force
            total +=1
        end
    end
    if (b.position[1] + axis_limit) < edge_radius
        plane_dist = [b.position[1] + axis_limit, 0, 0]
        if norm(plane_dist) < perceptionRadius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff.*egde_separation_force
            total +=1
        end
    end

    if (b.position[2] - axis_limit) < edge_radius
        plane_dist = [0, b.position[2] - axis_limit, 0]
        if norm(plane_dist) < perceptionRadius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff.*egde_separation_force
            total +=1
        end
    end
    if (b.position[2] + axis_limit) < edge_radius
        plane_dist = [0, b.position[2] + axis_limit, 0]
        if norm(plane_dist) < perceptionRadius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff.*egde_separation_force
            total +=1
        end
    end

    if (b.position[3] - axis_limit) < edge_radius
        plane_dist = [0, 0, b.position[3] - axis_limit]
        if norm(plane_dist) < perceptionRadius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff.*egde_separation_force
            total +=1
        end
    end
    if (b.position[3] + axis_limit) < edge_radius
        plane_dist = [0, 0, b.position[3] + axis_limit]
        if norm(plane_dist) < perceptionRadius
            diff = plane_dist / (norm(plane_dist)^2)
            steering += diff.*egde_separation_force
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

# follow the flocks average position 
function cohese!(b::Boid, flock::Vector{Boid})
    # max_force = 2.
    # max_speed = 5.
    perceptionRadius = 100 * perception_radius_multiplier
    steering = zeros(size(b.velocity))
    total = 0

    for boid in flock
        dist = b.position - boid.position
        if b.id != boid.id && norm(dist) < perceptionRadius
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
