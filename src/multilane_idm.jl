Base.@kwdef mutable struct MultiLaneIDMTargetsOverlay
    scene::EntityScene
    models::Dict{Int, AbstractMultiLaneIDM}
    color_front::Colorant = colorant"red"
    color_rear::Colorant = colorant"green"
    line_width::Float64 = 0.5
    arrowhead_len::Union{Nothing, Float64} = nothing
    textparams::TextParams = TextParams(y_start=20, size=20, color=colorant"white")
end

function AutomotiveVisualization.add_renderable!(
    rendermodel::RenderModel,
    overlay::MultiLaneIDMTargetsOverlay,
)
    for (id, model) in overlay.models
        render_multilaneidm(rendermodel, overlay, id, model)
    end
end

function render_arrows(
    rendermodel::RenderModel,
    overlay::MultiLaneIDMTargetsOverlay,
    veh_ego,
    targets
)
    for (id, (color, rear)) in targets
        veh_other = get_by_id(overlay.scene, id)
        render_arrow(rendermodel, veh_ego, veh_other, rear, color, overlay.line_width, overlay.arrowhead_len)
    end
end

function render_arrow(
    rendermodel::RenderModel,
    veh_ego,
    veh_other,
    rear::Bool,
    color::Colorant,
    line_width::Real,
    arrowhead_len,
)
    if rear
        A = VecE2(get_rear(veh_ego))
        B = VecE2(get_front(veh_other))
    else
        A = VecE2(get_front(veh_ego))
        B = VecE2(get_rear(veh_other))
    end
    render_arrow(rendermodel, A, B, color, line_width, arrowhead_len)
end

function render_arrow(
    rendermodel::RenderModel,
    A::VecE2,
    B::VecE2,
    color::Colorant,
    line_width::Real,
    arrowhead_len::Nothing,
)
    add_instruction!(
        rendermodel,
        render_line_segment,
        (A, B, color, line_width),
    )
end

function render_arrow(
    rendermodel::RenderModel,
    A::VecE2,
    B::VecE2,
    color::Colorant,
    line_width::Real,
    arrowhead_len::Real,
)
    add_instruction!(
        rendermodel,
        render_arrow,
        ([A B], color, line_width, arrowhead_len),
    )
end

function render_multilaneidm(
    rendermodel::RenderModel,
    overlay::MultiLaneIDMTargetsOverlay,
    id::Int,
    model::AbstractMultiLaneIDM,
)
    veh_ego = get_by_id(overlay.scene, id)
    targets = Dict()
    for id in model.target_style.front_target_ids
        targets[id] = (overlay.color_front, false)
    end
    for id in model.target_style.rear_target_ids
        targets[id] = (overlay.color_rear, true)
    end
    render_arrows(rendermodel, overlay, veh_ego, targets)
    return nothing
end

function render_multilaneidm(
    rendermodel::RenderModel,
    overlay::MultiLaneIDMTargetsOverlay,
    id::Int,
    model::FrenetMultiLaneIDM,
)
    veh_ego = get_by_id(overlay.scene, id)
    targets = Dict()
    for id in model.target_style.front_target_ids
        rear = false
        id ∈ keys(model.virtual_front_targets) || (targets[id] = (overlay.color_front, rear))
    end
    for id in model.target_style.rear_target_ids
        rear = true
        id ∈ keys(model.virtual_rear_targets) || (targets[id] = (overlay.color_rear, rear))
    end
    render_arrows(rendermodel, overlay, veh_ego, targets)
    return nothing
end

Base.@kwdef mutable struct MultiLaneIDMVirtualTargetsOverlay
    roadway::Roadway
    scene::EntityScene
    models::Dict{Int, AbstractMultiLaneIDM}
    color_front::Colorant = colorant"red"
    color_rear::Colorant = colorant"green"
    color_virtual::Colorant = colorant"yellow"
    line_width::Float64 = 0.5
    arrowhead_len::Union{Nothing, Float64} = nothing
end

function AutomotiveVisualization.add_renderable!(
    rendermodel::RenderModel,
    overlay::MultiLaneIDMVirtualTargetsOverlay,
)

    models = overlay.models
    for (id, model) in models
        ego = get_by_id(overlay.scene, id)
        render_virtual_targets(rendermodel, overlay, ego, model.virtual_front_targets, false)
        render_virtual_targets(rendermodel, overlay, ego, model.virtual_rear_targets, true)
    end
end

function render_virtual_targets(
    rendermodel::RenderModel,
    overlay::MultiLaneIDMVirtualTargetsOverlay,
    ego::Entity,
    virtual_targets,
    rear::Bool,
)
    target_point = rear ? VehicleTargetPointFront() : VehicleTargetPointRear()
    color = rear ? overlay.color_rear : overlay.color_front

    # target entity only created to get Δs
    target = Entity(VehicleState(VecSE2(0.,0.,0.), overlay.roadway, 0.0), VehicleDef(), 0)
    Δs = -targetpoint_delta(target_point, target)

    for (virt_id, virtual_target) in virtual_targets
        roadind = move_along(virtual_target.roadind, overlay.roadway, Δs)
        posg = overlay.roadway[roadind].pos
        target = Entity(VehicleState(posg, overlay.roadway, 0.0), VehicleDef(), virt_id)
        add_renderable!(
            rendermodel,
            EntityRectangle(entity=target, color=overlay.color_virtual
            ),
        )
        render_arrow(rendermodel, ego, target, rear, color, overlay.line_width, overlay.arrowhead_len)
    end
end

Base.@kwdef struct AccelerationOverlay
    ids::Vector{Int}
    actions::Scene{EntityAction{LaneFollowingAccel, Int}}
    textparams::TextParams = TextParams(size=15, x=700, y_start=50)
end

function AutomotiveVisualization.add_renderable!(
    rendermodel::RenderModel,
    overlay::AccelerationOverlay,
)
    textparams = overlay.textparams
    yₒ = textparams.y_start
    Δy = textparams.y_jump

    for (i, id) in enumerate(overlay.ids)
        action = get_by_id(overlay.actions, id)
        accel = action.action.a

        drawtext(
            @sprintf("ID %d: accel = % 1.1fm/s^2", id, accel),
            yₒ + (i - 1) * Δy,
            rendermodel,
            textparams,
        )
    end
end

Base.@kwdef struct SimulationOverlay
    nsim::Int
    t::Real
    textparams::TextParams = TextParams(size=15, x=200, y_start=50)
end

function AutomotiveVisualization.add_renderable!(
    rendermodel::RenderModel,
    overlay::SimulationOverlay,
)
    textparams = overlay.textparams
    yₒ = textparams.y_start

    drawtext(
        @sprintf("t = %2.1fs, nsim = %3d", overlay.t, overlay.nsim),
        yₒ,
        rendermodel,
        textparams,
    )
end

Base.@kwdef mutable struct RoadIndexOverlay
    roadway::Roadway
    roadind::RoadIndex
    color::Colorant = colorant"orange"
    circle_radius::Float64 = 1.0
end

function AutomotiveVisualization.add_renderable!(
    rendermodel::RenderModel,
    overlay::RoadIndexOverlay,
)
    posg = overlay.roadway[overlay.roadind].pos
    add_instruction!(
        rendermodel,
        render_circle,
        (posg.x, posg.y, overlay.circle_radius, overlay.color),
    )
end
