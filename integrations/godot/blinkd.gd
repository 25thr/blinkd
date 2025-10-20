extends Node
signal blink_detected(count: int, seconds: float)

@export var listen_port: int = 7777

var server := UDPServer.new()
var blink_count := 0
var start_time := Time.get_ticks_msec()

func _ready() -> void:
	var err = server.listen(listen_port)
	if err == OK:
		print("[Blinkd] Listening on UDP port %d..." % listen_port)
	else:
		push_error("[!] Failed to open UDP port %d (err %d)" % [listen_port, err])

func _process(delta: float) -> void:
	# Poll for new clients and packets
	server.poll()

	while server.is_connection_available():
		var peer: PacketPeerUDP = server.take_connection()
		if peer:
			# Retrieve the packet
			while peer.get_available_packet_count() > 0:
				var msg = peer.get_packet().get_string_from_utf8()
				_handle_message(msg)

func _handle_message(msg: String) -> void:
	var data = JSON.parse_string(msg)
	if typeof(data) == TYPE_DICTIONARY and data.has("type") and data["type"] == "blink":
		var dur_ms = data.get("dur_ms", 0)
		var flags = data.get("flags", 0)
		_trigger_blink_action(dur_ms, flags)

func _trigger_blink_action(dur_ms: int, flags: int) -> void:
	if flags & 1:  # Normal blink
		blink_count += 1
		var elapsed_sec = (Time.get_ticks_msec() - start_time) / 1000.0
		emit_signal("blink_detected", blink_count, elapsed_sec)
		print("[Blinkd] Blink #%d (%.2fs)" % [blink_count, elapsed_sec])
