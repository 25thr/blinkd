extends Node3D

@onready var sphere = $Sphere
@onready var label = $CanvasLayer/Label

var base_material: StandardMaterial3D
var blink_count := 0

func _ready():
	# Grab or create cube material
	base_material = sphere.get_active_material(0)
	if base_material == null:
		base_material = StandardMaterial3D.new()
		sphere.set_surface_override_material(0, base_material)

	# Connect the signal from the blinkd bridge
	Blinkd.connect("blink_detected", Callable(self, "_on_blink_detected"))

func _on_blink_detected(count: int, seconds: float) -> void:
	blink_count = count

	# Flash the material briefly
	var tween = create_tween()
	tween.tween_property(base_material, "emission", Color.RED, 0.2).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_OUT)
	tween.tween_property(base_material, "emission", Color.WHITE, 0.5).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN)

	# Update floating label text
	label.text = "Blinks: %d (%.2fs)" % [blink_count, seconds]
