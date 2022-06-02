using GLMakie
fig = Figure()
# AbstractPlotting.inline!(true) 

# create 3d axis
ax = Axis3(fig[1, 1], aspect = (1, 1, 1),
        title = "3d scatter")
        ax.perspectiveness = 0

# create button grid
fig[2, 1] = buttongrid = GridLayout(tellwidth = false)

labels = ["Run", "Step", "Restart"]

button_count = 3
# create buttons
buttons = buttongrid[1, 1:button_count] = [
    Button(fig,
        label = l, height = 60, width = 250, textsize = 30
    )
    for l in labels
]


# declare 1 point
points = Observable(Point3f[(0, 0, 0)])

# run button
on(buttons[1].clicks) do click
    frames = 1:30
    @time begin
    for i = frames
        points[] += Point3f[(0., 0.01, 0.)]
        sleep(0.01)
        # display(f)
    end
    end
    # record(fig, "append_animation.mp4", frames;
    #         framerate = 30) do frame
    #     points[] += Point3f[(0., 0.01, 0.)]
    #     sleep(0.05)
    # end
end

# step button - the same as run
on(buttons[2].clicks) do click
    points[] += Point3f[(0., 0.01, 0.)]
end

# reset button
on(buttons[3].clicks) do click
    points[] = Point3f[(0., 0., 0.)]
end

scatter!(ax, points, markersize=3000)



# animation 

# frames = 1:30
# record(fig, "append_animation.mp4", frames;
#         framerate = 30) do frame
    
#     # add new point
#     new_point = Point3f(frame/100, frame/100, frame/100)
#     points[] = push!(points[], new_point)
# end


fig