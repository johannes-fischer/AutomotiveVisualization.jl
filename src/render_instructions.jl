"""
Collection of basic rendering functions for Cairo
"""

Cairo.move_to(ctx::CairoContext, P::VecE2) = move_to(ctx, P.x, P.y)
Cairo.line_to(ctx::CairoContext, P::VecE2) = line_to(ctx, P.x, P.y)

function grayscale_transform(color::Colorant)
    g = 1.0 - gray(convert(Gray, color)) # convert to grayscale and invert
    if isa(color, TransparentColor)
        a = alpha(color)
        GrayA(g, a)
    else
        Gray(g)
    end
end

function Cairo.set_source_rgba(ctx::CairoContext, color::RGB)

    # g = convert(Float64, gray(grayscale_transform(color)))
    # r = g = b = g

    r = convert(Float64, red(color))
    g = convert(Float64, green(color))
    b = convert(Float64, blue(color))

    set_source_rgba(ctx, r, g, b, 1.0)
end
function Cairo.set_source_rgba(ctx::CairoContext, color::TransparentColor)

    # grayscale = grayscale_transform(color)
    # g = convert(Float64, gray(grayscale))
    # r = g = b = g
    # a = convert(Float64, alpha(grayscale))

    r = convert(Float64, red(color))
    g = convert(Float64, green(color))
    b = convert(Float64, blue(color))
    a = convert(Float64, alpha(color))

    set_source_rgba(ctx, r, g, b, a)
end
function Cairo.set_source_rgba(ctx::CairoContext, color₀::Colorant, color₁::Colorant, t::Real)

    r₀ = convert(Float64, red(color₀))
    g₀ = convert(Float64, green(color₀))
    b₀ = convert(Float64, blue(color₀))
    a₀ = convert(Float64, alpha(color₀))

    r₁ = convert(Float64, red(color₁))
    g₁ = convert(Float64, green(color₁))
    b₁ = convert(Float64, blue(color₁))
    a₁ = convert(Float64, alpha(color₁))

    r = r₀ + (r₁ - r₀)*t
    g = g₀ + (g₁ - g₀)*t
    b = b₀ + (b₁ - b₀)*t
    a = a₀ + (a₁ - a₀)*t
    set_source_rgba(ctx, r, g, b, a)
end


function render_paint(
    ctx          :: CairoContext,
    color        :: Colorant,
    )

    r = convert(Float64, red(color))
    g = convert(Float64, green(color))
    b = convert(Float64, blue(color))
    a = convert(Float64, alpha(color))

    save(ctx)
    set_source_rgba(ctx, color)
    paint_with_alpha(ctx, alpha(color))
    restore(ctx)
end

function render_text(
    ctx          :: CairoContext,
    text         :: AbstractString,
    x            :: Real,
    y            :: Real,
    fontsize     :: Real,
    color        :: Colorant,
    align_center :: Bool        = false,
    fontfamily   :: AbstractString      = "monospace" # ∈ "serif", "sans-serif", "cursive", "fantasy", "monospace"
    )

    save(ctx)
    select_font_face(ctx, fontfamily, Cairo.FONT_SLANT_NORMAL, Cairo.FONT_WEIGHT_NORMAL)
    set_font_size(ctx, fontsize)
    set_source_rgba(ctx, color)

    if align_center
        extents = text_extents(ctx, text)
        x -= (extents[3]/2 + extents[1]);
        y -= (extents[4]/2 + extents[2]);
    end

    move_to(ctx, x, y)
    show_text(ctx, text)
    restore(ctx)
end

function render_circle(
    ctx          :: CairoContext,
    x            :: Real,
    y            :: Real,
    radius       :: Real,
    color_fill   :: Colorant,
    color_stroke :: Colorant = color_fill,
    line_width   :: Real = 1.0
    )

    save(ctx)
    translate(ctx, x, y)

    arc(ctx, 0.0, 0.0, radius, 0, 2pi)
    set_source_rgba(ctx, color_fill)
    fill(ctx)

    arc(ctx, 0.0, 0.0, radius, 0, 2pi)
    set_line_width(ctx, line_width)
    set_source_rgba(ctx, color_stroke)
    stroke(ctx)

    restore(ctx)
end
function render_circle(
    ctx          :: CairoContext,
    P            :: VecE2,
    radius       :: Real,
    color_fill   :: Colorant,
    color_stroke :: Colorant = color_fill,
    line_width   :: Real = 1.0
    )

    render_circle(ctx, P.x, P.y, radius, color_fill, color_stroke, line_width)
end
function render_arc(
    ctx          :: CairoContext,
    x            :: Real,
    y            :: Real,
    radius       :: Real,
    start_ang    :: Real,
    end_ang      :: Real,
    color_fill   :: Colorant,
    color_stroke :: Colorant = color_fill,
    line_width   :: Real = 1.0
    )

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    translate(ctx, x, y)

    arc(ctx, 0.0, 0.0, radius, start_ang, end_ang)
    set_source_rgba(ctx, color_fill)
    fill(ctx)

    arc(ctx, 0.0, 0.0, radius, start_ang, end_ang)
    set_line_width(ctx, line_width)
    set_source_rgba(ctx, color_stroke)
    stroke(ctx)

    restore(ctx)
end
function render_rect(ctx::CairoContext,
                    x::Real,
                    y::Real,
                    width::Real,
                    height::Real,
                    color_fill::Colorant,
                    ffill         :: Bool        = false,
                    fstroke       :: Bool        = true,
                    color_stroke  :: Colorant    = color_fill,
                    line_width    :: Real        = 2.0
                    )
    Cairo.save(ctx)
    rectangle(ctx, x, y, width, height)
    if ffill
        set_source_rgba(ctx, color_fill)
        fstroke ? fill_preserve(ctx) : fill(ctx)
    end

    if fstroke
        set_source_rgba(ctx, color_stroke)
        set_line_width(ctx, line_width)
        stroke(ctx)
    end

    restore(ctx)
end
function render_round_rect(
    ctx           :: CairoContext,
    x             :: Real,
    y             :: Real,
    width         :: Real,
    height        :: Real,
    aspect        :: Real,
    corner_radius :: Real,
    color_fill    :: Colorant,
    ffill         :: Bool        = false,
    fstroke       :: Bool        = true,
    color_stroke  :: Colorant    = color_fill,
    line_width    :: Real        = 2.0
    )
    # (x,y) are the center of the rectangle

    save(ctx)
    radius = corner_radius / aspect

    d2r = pi/180

    new_sub_path(ctx)
    arc(ctx, x + width/2 - radius, y - height/2 + radius, radius, -90 * d2r,   0 * d2r)
    arc(ctx, x + width/2 - radius, y + height/2 - radius, radius,   0 * d2r,  90 * d2r)
    arc(ctx, x - width/2 + radius, y + height/2 - radius, radius,  90 * d2r, 180 * d2r)
    arc(ctx, x - width/2 + radius, y - height/2 + radius, radius, 180 * d2r, 270 * d2r)
    close_path(ctx)

    if ffill
        set_source_rgba(ctx, color_fill)
        fstroke ? fill_preserve(ctx) : fill(ctx)
    end

    if fstroke
        set_source_rgba(ctx, color_stroke)
        # TODO: bug - line_width seems to be changing based on vehicle heading angle
        # from Cairo.user_to_device_distance! - this may be the bug
        line_width = abs(user_to_device_distance!(ctx, [line_width,0.0])[1])
        set_line_width(ctx, line_width)
        stroke(ctx)
    end

    restore(ctx)
end
function render_car(
    ctx           :: CairoContext,
    x             :: Real, # center of vehicle [m]
    y             :: Real, # center of vehicle [m]
    yaw           :: Real, # counterclockwise [rad]
    color_fill    :: Colorant,
    color_stroke  :: Colorant = color_fill;

    color_arrow   :: Colorant = RGB(1.0,1.0,1.0),
    car_length    :: Float64 = 4.6, # [m]
    car_width     :: Float64 = 2.0, # [m]
    corner_radius :: Float64 = 0.3, # [m]
    corner_aspect :: Float64 = 1.0, # [m]
    arrow_width   :: Float64 = 0.3, # [% car width]
    arrow_length  :: Float64 = 0.6, # [% car length]
    arrow_headlen :: Float64 = 0.5, # [% arrow len that head takes up]
    line_width    :: Float64 = 0.1, # [m]
    )

    # renders a car (rounded rectangle w/ arrow) at the given location

    save(ctx)

    translate(ctx, x, y)
    rotate(ctx, yaw)

    render_round_rect(ctx, 0, 0, car_length, car_width, corner_aspect, corner_radius, color_fill, true, true, color_stroke, line_width)

    # render the arrow
    wid = car_width*arrow_width
    len = car_length*arrow_length
    hed = len*arrow_headlen

    new_sub_path(ctx)
    move_to(ctx, 0+len/2,     0      )
    line_to(ctx, 0+len/2-hed, 0+hed/2)
    line_to(ctx, 0+len/2-hed, 0+wid/2)
    line_to(ctx, 0-len/2,     0+wid/2)
    line_to(ctx, 0-len/2,     0-wid/2)
    line_to(ctx, 0+len/2-hed, 0-wid/2)
    line_to(ctx, 0+len/2-hed, 0-hed/2)
    close_path(ctx)

    set_source_rgba(ctx, color_arrow)
    fill(ctx)

    restore(ctx)
end
function render_vehicle(
    ctx           :: CairoContext,
    x             :: Real, # x-pos of the center of the vehicle
    y             :: Real, # y-pos of the center of the vehicle
    yaw           :: Real, # heading angle [rad]
    length        :: Real, # vehicle length
    width         :: Real, # vehicle width
    color_fill    :: Colorant,
    color_stroke  :: Colorant = color_fill,

    color_arrow   :: Colorant = RGB(1.0,1.0,1.0),
    corner_radius :: Float64 = 0.5,
    corner_aspect :: Float64 = 1.0,
    arrow_width   :: Float64 = 0.3, # [% car width]
    arrow_length  :: Float64 = 0.6, # [% car length]
    arrow_headlen :: Float64 = 0.5, # [% arrow len that head takes up]
    line_width    :: Float64 = 0.3,
    )

    # renders a car (rounded rectangle w/ arrow) at the given location
    # (x,y) are in meters and yaw is the radians, counter-clockwise from pos x axis

    save(ctx)

    # translate(ctx, x - 0.5length*cos(yaw), y - 0.5length*sin(yaw))
    translate(ctx, x, y)
    rotate(ctx, yaw)

    render_round_rect(ctx, 0, 0, length, width, corner_aspect, corner_radius, color_fill, true, true, color_stroke, line_width)

    # render the arrow
    wid = width*arrow_width
    len = length*arrow_length
    hed = min(len*arrow_headlen, width)

    new_sub_path(ctx)
    move_to(ctx, 0+len/2,     0      )
    line_to(ctx, 0+len/2-hed, 0+hed/2)
    line_to(ctx, 0+len/2-hed, 0+wid/2)
    line_to(ctx, 0-len/2,     0+wid/2)
    line_to(ctx, 0-len/2,     0-wid/2)
    line_to(ctx, 0+len/2-hed, 0-wid/2)
    line_to(ctx, 0+len/2-hed, 0-hed/2)
    close_path(ctx)

    set_source_rgba(ctx, color_arrow)
    fill(ctx)

    restore(ctx)
end

# aggregate
function render_point_trail(
    ctx           :: CairoContext,
    pts           :: AbstractVector{T}, # 2×n
    color         :: Colorant,
    circle_radius :: Real = 0.25 ) where {T<:Real}

    save(ctx)

    for i = 1 : size(pts,2)
        arc(ctx, pts[1,i], pts[2,i], circle_radius, 0, 2pi)
        set_line_width(ctx, 1.0)
        set_source_rgba(ctx, color)
        stroke(ctx)
    end

    restore(ctx)
end
function render_line(
    ctx        :: CairoContext,
    pts        :: AbstractArray{T}, # 2×n
    color      :: Colorant,
    line_width :: Real = 1.0,
    line_cap   :: Integer=Cairo.CAIRO_LINE_CAP_ROUND, # CAIRO_LINE_CAP_BUTT, CAIRO_LINE_CAP_ROUND, CAIRO_LINE_CAP_SQUARE
    ) where {T<:Real}

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,line_width)
    set_line_cap(ctx, line_cap)

    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    stroke(ctx)
    restore(ctx)
end
function render_line(
    ctx        :: CairoContext,
    pts        :: AbstractVector{VecE2{T}}, # 2×n
    color      :: Colorant,
    line_width :: Real = 1.0,
    line_cap   :: Integer=Cairo.CAIRO_LINE_CAP_ROUND, # CAIRO_LINE_CAP_BUTT, CAIRO_LINE_CAP_ROUND, CAIRO_LINE_CAP_SQUARE
    ) where {T<:Real}

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,line_width)
    set_line_cap(ctx, line_cap)

    move_to(ctx, pts[1])
    for i in 2 : length(pts)
        line_to(ctx, pts[i])
    end
    stroke(ctx)
    restore(ctx)
end
function render_closed_line(
    ctx        :: CairoContext,
    pts        :: AbstractArray{T}, # 2×n
    color      :: Colorant,
    line_width :: Real = 1.0,
    fill_color :: Colorant = RGBA(0.0,0.0,0.0,0.0),
    ) where {T<:Real}

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)

    set_source_rgba(ctx, color)
    set_line_width(ctx,line_width)

    move_to(ctx, pts[1,1], pts[2,1])
    for i in 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    close_path(ctx)

    if alpha(fill_color) > 0.0
        stroke_preserve(ctx)
        set_source_rgba(ctx, fill_color)
        fill(ctx)
    else
        stroke(ctx)
    end

    restore(ctx)
end
function render_closed_line(
    ctx        :: CairoContext,
    pts        :: AbstractVector{VecE2{T}},
    color      :: Colorant,
    line_width :: Real = 1.0,
    fill_color :: Colorant = RGBA(0.0,0.0,0.0,0.0),
    ) where {T<:Real}

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)

    set_source_rgba(ctx, color)
    set_line_width(ctx,line_width)

    move_to(ctx, pts[1])
    for i in 2 : length(pts)
        line_to(ctx, pts[i])
    end
    close_path(ctx)

    if alpha(fill_color) > 0.0
        stroke_preserve(ctx)
        set_source_rgba(ctx, fill_color)
        fill(ctx)
    else
        stroke(ctx)
    end

    restore(ctx)
end
function render_fill_region(
    ctx        :: CairoContext,
    pts        :: AbstractArray{T}, # 2×n
    color      :: Colorant,
    ) where {T<:Real}

    save(ctx)
    set_source_rgba(ctx,color)
    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    close_path(ctx)
    fill(ctx)
    restore(ctx)

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,1.0)
    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    close_path(ctx)
    stroke(ctx)
    restore(ctx)
end
function render_line_segment(
    ctx        :: CairoContext,
    x1         :: Float64,
    y1         :: Float64,
    x2         :: Float64,
    y2         :: Float64,
    color      :: Colorant,
    line_width :: Real = 1.0
    )

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,line_width)
    set_line_cap(ctx, Cairo.CAIRO_LINE_CAP_ROUND)

    move_to(ctx, x1, y1)
    line_to(ctx, x2, y2)
    stroke(ctx)
    restore(ctx)
end
function render_line_segment(
    ctx        :: CairoContext,
    A          :: VecE2,
    B          :: VecE2,
    color      :: Colorant,
    line_width :: Real = 1.0
    )

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,line_width)
    set_line_cap(ctx, Cairo.CAIRO_LINE_CAP_ROUND)

    move_to(ctx, A)
    line_to(ctx, B)
    stroke(ctx)
    restore(ctx)
end
function render_dashed_line(
    ctx          :: CairoContext,
    pts          :: AbstractArray{T}, # 2×n
    color        :: Colorant,
    line_width_in   :: Real = 1.0,
    dash_length_in  :: Real = 1.0,
    dash_spacing_in :: Real = 1.0,
    dash_offset_in  :: Real = 0.0,
    line_cap :: Integer = Cairo.CAIRO_LINE_CAP_SQUARE ,
    ) where {T<:Real}

    line_width   = user_to_device_distance!(ctx, [line_width_in,  0])[1]
    dash_length  = user_to_device_distance!(ctx, [dash_length_in, 0])[1]
    dash_spacing = user_to_device_distance!(ctx, [dash_spacing_in,0])[1]
    dash_offset  = user_to_device_distance!(ctx, [dash_offset_in, 0])[1]
    dashes = [dash_length, dash_spacing]

    save(ctx)
    set_source_rgba(ctx,color)
    set_line_width(ctx,line_width)
    set_line_cap(ctx, convert(Int32, line_cap))
    set_dash(ctx, dashes, dash_offset)

    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    stroke(ctx)
    restore(ctx)
end
function render_dashed_arc(
    ctx          :: CairoContext,
    x            :: Real,
    y            :: Real,
    radius       :: Real,
    start_ang    :: Real,
    end_ang      :: Real,
    color_fill   :: Colorant,
    color_stroke :: Colorant   = color_fill,
    line_width_in   :: Real = 1.0,
    dash_length_in  :: Real = 1.0,
    dash_spacing_in :: Real = 1.0,
    dash_offset_in  :: Real = 0.0
    )

    line_width   = user_to_device_distance!(ctx, [line_width_in,  0])[1]
    dash_length  = user_to_device_distance!(ctx, [dash_length_in, 0])[1]
    dash_spacing = user_to_device_distance!(ctx, [dash_spacing_in,0])[1]
    dash_offset  = user_to_device_distance!(ctx, [dash_offset_in, 0])[1]
    dashes = [dash_length, dash_spacing]

    save(ctx)
    translate(ctx, x, y)

    arc(ctx, 0.0, 0.0, radius, start_ang, end_ang)
    set_source_rgba(ctx, color_fill)
    fill(ctx)

    arc(ctx, 0.0, 0.0, radius, start_ang, end_ang)
    set_source_rgba(ctx, color_stroke)
    set_line_width(ctx, line_width)
    set_line_cap(ctx, Cairo.CAIRO_LINE_CAP_ROUND)
    set_dash(ctx, dashes, dash_offset)
    stroke(ctx)

    restore(ctx)
end
function render_arrow(
    ctx               :: CairoContext,
    pts               :: AbstractArray{T}, # 2×n
    color             :: Colorant,
    line_width        :: Real,
    arrowhead_len     :: Real;
    ARROW_WIDTH_RATIO :: AbstractFloat = 0.8,
    ARROW_ALPHA       :: AbstractFloat = 0.1pi,
    ARROW_BETA        :: AbstractFloat = 0.8pi
    ) where {T<:Real}

    @assert(size(pts,2) > 1)

    line_width = user_to_device_distance!(ctx, [line_width,0])[1]

    save(ctx)
    set_source_rgba(ctx, color)
    set_line_width(ctx, line_width)
    set_line_cap(ctx, Cairo.CAIRO_LINE_CAP_ROUND)

    move_to(ctx, pts[1,1], pts[2,1])
    for i = 2 : size(pts,2)
        line_to(ctx, pts[1,i], pts[2,i])
    end
    stroke(ctx)

    # orientation of the arrowhead
    theta = atan(pts[2,end]-pts[2,end-1], pts[1,end]-pts[1,end-1])

    arrowhead_width = arrowhead_len * ARROW_WIDTH_RATIO
    whatev =    pi - ARROW_BETA
    delta = 0.5pi - whatev
    epsilon =  pi - ARROW_BETA - ARROW_ALPHA
    len_prime = 0.5arrowhead_width*sin(delta + epsilon)/sin(ARROW_ALPHA) - arrowhead_len

    R = [ cos(theta) -sin(theta);
          sin(theta)  cos(theta) ]

    # render the arrowhead
    O = pts[:,end] # origin, center of the arrowhead
    A = O + reshape(R * [ arrowhead_len/2 0]', 2)
    B = O + reshape(R * [-arrowhead_len/2 0]', 2)
    C = O + reshape(R * [-arrowhead_len/2-len_prime  arrowhead_width/2]', 2)
    D = O + reshape(R * [-arrowhead_len/2-len_prime -arrowhead_width/2]', 2)

    new_sub_path(ctx)
    move_to(ctx, A[1], A[2])
    line_to(ctx, C[1], C[2])
    line_to(ctx, B[1], B[2])
    line_to(ctx, D[1], D[2])
    close_path(ctx)
    fill(ctx)

    restore(ctx)
end

function render_colormesh(
    ctx::CairoContext,
    C::AbstractArray{T}, # n×m matrix of 0->1 values
    X::AbstractVector{S}, # n+1 vector of x bin boundaries
    Y::AbstractVector{U} # m+1 vector of y bin boundaries
    ) where {T<:Real, S<:Real, U<:Real}

    n,m = size(C)
    @assert(length(X) == n+1)
    @assert(length(Y) == n+1)

    save(ctx)

    x₂ = X[1]
    for i = 1 : n
        x₁, x₂ = x₂, X[i+1]
        y₂ = Y[1]
        for j = 1 : m
            y₁, y₂ = y₂, Y[j+1]

            c = C[i,j]

            new_sub_path(ctx)
            move_to(ctx, x₁, y₁)
            line_to(ctx, x₂, y₁)
            line_to(ctx, x₂, y₂)
            line_to(ctx, x₁, y₂)
            close_path(ctx)

            set_source_rgba(ctx, c, c, c, 1.0)
            fill(ctx)
        end
    end

    restore(ctx)
end
function render_colormesh(
    ctx::CairoContext,
    C::AbstractArray{T}, # n×m matrix of 0->1 values
    X::AbstractVector{S}, # n+1 vector of x bin boundaries
    Y::AbstractVector{U}, # m+1 vector of y bin boundaries
    color₀::Colorant, # color for c = 0
    color₁::Colorant, # color for c = 1
    ) where {T<:Real, S<:Real, U<:Real}


    n,m = size(C)
    @assert(length(X) == n+1)
    @assert(length(Y) == m+1)

    save(ctx)

    x₂ = X[1]
    for i = 1 : n
        x₁, x₂ = x₂, X[i+1]
        y₂ = Y[1]
        for j = 1 : m
            y₁, y₂ = y₂, Y[j+1]

            set_source_rgba(ctx, color₀, color₁, C[i,j])

            new_sub_path(ctx)
            move_to(ctx, x₁, y₁)
            line_to(ctx, x₂, y₁)
            line_to(ctx, x₂, y₂)
            line_to(ctx, x₁, y₂)
            close_path(ctx)

            fill(ctx)
        end
    end

    restore(ctx)
end
