# Flash Storage Plan for Demo Waypoints

## Current Situation

### Demo Recording State (Current - RAM Only)
From [host_comms.c](app/src/comms/host_comms.c):
```c
#define MAX_DEMO_WAYPOINTS 50

struct demo_waypoint {
    float joint_angles[6];  // 6 × 4 bytes = 24 bytes
    uint32_t delay_ms;      // 4 bytes
};                          // Total: 28 bytes per waypoint

static struct {
    bool is_recording;
    uint8_t demo_id;        // 0-2 (3 demo slots)
    uint8_t waypoint_count;
    struct demo_waypoint waypoints[MAX_DEMO_WAYPOINTS]; // 1400 bytes
} demo_recording;
```

**Problem**: Lost on reset, only one demo in RAM at a time

---

## Flash Storage Architecture

### 1. Available Flash Partition

From [m5stack_atom_lite.dts](boards/m5stack_atom_lite/m5stack_atom_lite.dts):
```dts
storage_partition: partition@3b0000 {
    label = "storage";
    reg = <0x3B0000 0x30000>;  // 192 KB starting at 0x3B0000
};
```

**Available**: 192 KB (196,608 bytes) for all user data storage

### 2. Storage Layout Design

#### Option A: Raw Flash API (Lower Level)
**Pros**: Direct control, minimal overhead, no wear leveling complexity
**Cons**: Manual sector management, no automatic wear leveling, corruption risk

#### Option B: NVS (Non-Volatile Storage)
**Pros**: Automatic wear leveling, CRC validation, key-value API
**Cons**: Slightly higher overhead, more complex initialization

#### Option C: Flash Stream
**Pros**: Simple append-only API, good for sequential writes
**Cons**: Less suitable for random access updates

**Recommendation**: **NVS** - Best balance for demo storage (few writes, need reliability)

### 3. Flash Layout with NVS

```
Storage Partition (192 KB @ 0x3B0000)
├─ NVS Area (16 KB)                    ← NVS metadata + wear leveling
│  └─ Key-value pairs for demos
│
├─ Demo Storage (48 KB)                ← 3 demos × 16 KB each
│  ├─ Demo Slot 0 (16 KB)
│  │  ├─ Header (64 bytes)
│  │  │  ├─ Magic (4B): 0xDEMO0001
│  │  │  ├─ Version (1B): 0x01
│  │  │  ├─ Demo ID (1B): 0
│  │  │  ├─ Waypoint Count (1B): 0-50
│  │  │  ├─ Reserved (1B)
│  │  │  ├─ Timestamp (4B): Unix time
│  │  │  ├─ Name (32B): Null-terminated string
│  │  │  ├─ Description (16B): Optional
│  │  │  └─ CRC32 (4B): Header checksum
│  │  │
│  │  └─ Waypoints (1400 bytes max)
│  │     └─ 50 × demo_waypoint (28B each)
│  │
│  ├─ Demo Slot 1 (16 KB)
│  └─ Demo Slot 2 (16 KB)
│
└─ Reserved (128 KB)                   ← Future: trajectories, configs, calibration
```

**Calculation**:
- Header: 64 bytes
- Waypoints: 50 × 28 = 1,400 bytes
- Total per demo: 1,464 bytes
- With padding to 2 KB alignment: ~2 KB per demo
- Actual slot size: 16 KB (plenty of headroom for future features)

---

## API Design

### Module: `demo_storage.h` / `demo_storage.c`

#### Data Structures

```c
/* Demo metadata header (64 bytes, flash-aligned) */
struct demo_header {
    uint32_t magic;              // 0xDEMO0001
    uint8_t version;             // 0x01
    uint8_t demo_id;             // 0-2
    uint8_t waypoint_count;      // 0-50
    uint8_t flags;               // Bit 0: valid, Bit 1: locked
    uint32_t timestamp;          // Unix timestamp
    char name[32];               // User-friendly name
    char description[16];        // Optional description
    uint32_t crc32;              // CRC32 of header + waypoints
};

/* Complete demo structure (in-memory representation) */
struct demo_sequence {
    struct demo_header header;
    struct demo_waypoint waypoints[MAX_DEMO_WAYPOINTS];
};
```

#### Public API Functions

##### Initialization
```c
/**
 * @brief Initialize demo storage subsystem
 * 
 * - Opens flash device
 * - Initializes NVS if using NVS backend
 * - Validates storage partition
 * - Performs integrity check on all demo slots
 * 
 * @return 0 on success, negative errno on failure
 */
int demo_storage_init(void);
```

##### Write Operations
```c
/**
 * @brief Save a demo to flash
 * 
 * Writes demo header + waypoints to the specified slot.
 * Automatically calculates CRC32 for integrity.
 * 
 * @param demo_id Demo slot (0-2)
 * @param waypoints Array of waypoints
 * @param count Number of waypoints (1-50)
 * @param name Optional demo name (max 31 chars + null)
 * @return 0 on success, negative errno on failure
 *         -EINVAL: Invalid demo_id or count
 *         -ENOMEM: Too many waypoints
 *         -EIO: Flash write error
 */
int demo_storage_save(uint8_t demo_id, 
                      const struct demo_waypoint *waypoints,
                      uint8_t count,
                      const char *name);

/**
 * @brief Update demo metadata (name/description) without changing waypoints
 * 
 * @param demo_id Demo slot (0-2)
 * @param name New name (NULL to keep existing)
 * @param description New description (NULL to keep existing)
 * @return 0 on success, negative errno on failure
 */
int demo_storage_update_metadata(uint8_t demo_id,
                                  const char *name,
                                  const char *description);
```

##### Read Operations
```c
/**
 * @brief Load a demo from flash
 * 
 * Reads demo header + waypoints, validates CRC32.
 * 
 * @param demo_id Demo slot (0-2)
 * @param waypoints Output buffer (must hold MAX_DEMO_WAYPOINTS)
 * @param count Output: actual waypoint count
 * @return 0 on success, negative errno on failure
 *         -EINVAL: Invalid demo_id
 *         -ENOENT: Demo slot empty
 *         -EILSEQ: CRC mismatch (corrupted data)
 */
int demo_storage_load(uint8_t demo_id,
                      struct demo_waypoint *waypoints,
                      uint8_t *count);

/**
 * @brief Read just the demo header (metadata only)
 * 
 * Fast query to check if demo exists and get name/timestamp/count
 * without loading all waypoints.
 * 
 * @param demo_id Demo slot (0-2)
 * @param header Output buffer for header
 * @return 0 on success, negative errno on failure
 */
int demo_storage_read_header(uint8_t demo_id,
                              struct demo_header *header);

/**
 * @brief Get list of all stored demos
 * 
 * @param info Output array (size 3)
 * @return Number of valid demos found (0-3)
 */
int demo_storage_get_list(struct demo_info {
    uint8_t demo_id;
    char name[32];
    uint8_t waypoint_count;
    uint32_t timestamp;
    bool is_valid;
} info[3]);
```

##### Delete/Maintenance Operations
```c
/**
 * @brief Clear a demo slot (mark as empty)
 * 
 * @param demo_id Demo slot (0-2)
 * @return 0 on success, negative errno on failure
 */
int demo_storage_clear(uint8_t demo_id);

/**
 * @brief Erase all demos (factory reset)
 * 
 * WARNING: Cannot be undone!
 * 
 * @return 0 on success, negative errno on failure
 */
int demo_storage_erase_all(void);

/**
 * @brief Verify integrity of a demo (CRC check)
 * 
 * @param demo_id Demo slot (0-2)
 * @return 0 if valid, -EILSEQ if corrupted, -ENOENT if empty
 */
int demo_storage_verify(uint8_t demo_id);

/**
 * @brief Get storage statistics
 * 
 * @param stats Output structure
 * @return 0 on success
 */
int demo_storage_get_stats(struct demo_storage_stats {
    uint32_t total_bytes;          // Total storage size
    uint32_t used_bytes;           // Bytes used by demos
    uint32_t available_bytes;      // Free space
    uint8_t demo_count;            // Number of stored demos
    uint32_t write_count;          // Total writes (wear tracking)
    uint32_t erase_count;          // Total erases
} *stats);
```

---

## Implementation Strategy

### Phase 1: Flash HAL Wrapper
**File**: `app/src/hal/hal_flash.c`

```c
/* Thin wrapper around Zephyr flash API */
int hal_flash_init(void);
int hal_flash_read(uint32_t offset, void *data, size_t len);
int hal_flash_write(uint32_t offset, const void *data, size_t len);
int hal_flash_erase(uint32_t offset, size_t len);
```

### Phase 2: Storage Layer
**File**: `app/src/storage/demo_storage.c`

#### Initialization
```c
int demo_storage_init(void) {
    1. Get flash device: DEVICE_DT_GET(DT_NODELABEL(storage_partition))
    2. Check device ready
    3. Validate partition size
    4. Initialize NVS (if using)
    5. Scan all 3 demo slots for valid headers
    6. Log storage statistics
}
```

#### Save Operation
```c
int demo_storage_save(...) {
    1. Validate inputs (demo_id 0-2, count 1-50)
    2. Build demo_header (magic, version, timestamp, name)
    3. Copy waypoints to buffer
    4. Calculate CRC32 over header + waypoints
    5. Erase flash sector for demo slot
    6. Write header (64 bytes aligned)
    7. Write waypoints (28 × count bytes)
    8. Verify write (read-back + CRC check)
    9. Return status
}
```

#### Load Operation
```c
int demo_storage_load(...) {
    1. Calculate flash offset for demo_id
    2. Read header (64 bytes)
    3. Validate magic number (0xDEMO0001)
    4. Check waypoint_count <= 50
    5. Read waypoints (28 × count bytes)
    6. Compute CRC32
    7. Compare with stored CRC
    8. Return waypoints if valid, error if corrupted
}
```

#### Clear Operation
```c
int demo_storage_clear(...) {
    1. Calculate flash offset
    2. Erase sector (16 KB)
    3. Optionally write empty header with magic=0
}
```

### Phase 3: Integration with host_comms
**Update**: `app/src/comms/host_comms.c`

```c
/* In handle_finish_demo_recording() */
static int handle_finish_demo_recording(void) {
    if (!demo_recording.is_recording) {
        return -EINVAL;
    }
    
    // NEW: Save to flash
    int ret = demo_storage_save(
        demo_recording.demo_id,
        demo_recording.waypoints,
        demo_recording.waypoint_count,
        NULL  // Auto-generate name like "Demo 0"
    );
    
    if (ret != 0) {
        LOG_ERR("Failed to save demo to flash: %d", ret);
        return ret;
    }
    
    LOG_INF("Demo %d saved to flash (%d waypoints)",
            demo_recording.demo_id, demo_recording.waypoint_count);
    
    demo_recording.is_recording = false;
    return 0;
}

/* In handle_play_demo() */
static int handle_play_demo(...) {
    uint8_t demo_id = payload[0];
    
    // NEW: Load from flash
    struct demo_waypoint waypoints[MAX_DEMO_WAYPOINTS];
    uint8_t count;
    
    int ret = demo_storage_load(demo_id, waypoints, &count);
    if (ret != 0) {
        LOG_ERR("Failed to load demo %d: %d", demo_id, ret);
        return ret;
    }
    
    LOG_INF("Playing demo %d (%d waypoints)", demo_id, count);
    
    // Execute waypoints...
    for (uint8_t i = 0; i < count; i++) {
        // ... existing playback code ...
    }
}

/* In handle_clear_demo() */
static int handle_clear_demo(...) {
    uint8_t demo_id = payload[0];
    
    // NEW: Erase from flash
    int ret = demo_storage_clear(demo_id);
    if (ret != 0) {
        LOG_ERR("Failed to clear demo %d: %d", demo_id, ret);
        return ret;
    }
    
    // Also clear RAM if it matches
    if (demo_id == demo_recording.demo_id) {
        demo_recording.waypoint_count = 0;
    }
    
    return 0;
}
```

---

## Configuration (prj.conf)

### For Raw Flash API
```ini
# Flash driver
CONFIG_FLASH=y
CONFIG_FLASH_PAGE_LAYOUT=y
CONFIG_FLASH_MAP=y

# CRC for integrity
CONFIG_CRC=y
```

### For NVS (Recommended)
```ini
# NVS subsystem
CONFIG_NVS=y
CONFIG_NVS_LOG_LEVEL_INF=y

# Flash requirements
CONFIG_FLASH=y
CONFIG_FLASH_PAGE_LAYOUT=y
CONFIG_FLASH_MAP=y
CONFIG_MPU_ALLOW_FLASH_WRITE=y

# CRC for integrity
CONFIG_CRC=y
```

---

## Testing Strategy

### Unit Tests
1. **Storage Init**: Verify partition accessible
2. **Save/Load Round-trip**: Write demo, read back, compare
3. **CRC Validation**: Corrupt data, verify detection
4. **Boundary Conditions**: 
   - Empty slot
   - Full slot (50 waypoints)
   - Invalid demo_id
5. **Multiple Demos**: Save 3 demos, load each, verify independence
6. **Clear**: Write demo, clear, verify empty
7. **Power-loss Simulation**: Interrupt write mid-operation, verify recovery

### Integration Tests
1. Record demo via host_comms → verify in flash
2. Power cycle → playback demo from flash
3. Record multiple demos → list all → playback each
4. Clear one demo → verify others unaffected

### Stress Tests
1. **Wear Test**: Write/erase 1000 cycles (check NVS wear leveling)
2. **Full Storage**: Write max waypoints to all 3 slots
3. **Concurrent Access**: Ensure no race conditions (if future threads added)

---

## Performance Considerations

### Flash Write Times (ESP32)
- **Erase sector (4KB)**: ~50-100 ms
- **Write 64 bytes**: ~1-2 ms
- **Total save time**: ~100-150 ms per demo

**Impact**: Not critical for demo recording (happens once, user-initiated)

### Flash Endurance (ESP32 Internal Flash)
- **Typical**: 10,000 - 100,000 erase cycles
- **Demo usage**: ~10-100 saves per slot over lifetime
- **Conclusion**: Well within limits, no special wear leveling needed beyond NVS

### Memory Usage
- **RAM impact**: +64 bytes (demo_header buffer during operations)
- **Code size**: ~2-3 KB for storage module
- **Flash usage**: 48 KB reserved (3 × 16 KB slots)

---

## Error Handling

### Corruption Detection
1. **Magic number check**: 0xDEMO0001 must match
2. **CRC32 validation**: Full header + waypoints
3. **Bounds check**: waypoint_count <= 50
4. **Version check**: Future-proof for format changes

### Recovery Strategy
- **Corrupted slot**: Mark invalid, allow user to clear
- **Partial write**: Next write erases sector first (auto-recovery)
- **All slots corrupted**: Factory reset function available

### Logging
```c
LOG_ERR("Demo %d CRC mismatch: expected 0x%08X, got 0x%08X", id, expected, actual);
LOG_WRN("Demo %d has invalid waypoint count: %d (max %d)", id, count, MAX);
LOG_INF("Demo %d saved successfully (%d waypoints, %u bytes)", id, count, size);
```

---

## Future Enhancements (Post-MVP)

### Phase 4+
1. **Named Demos**: User-provided names (already in header structure)
2. **Demo Export/Import**: Serialize to host for backup/sharing
3. **Demo Compression**: LZ77 for angle sequences (save space)
4. **Trajectory Interpolation**: Store keyframes, compute intermediate points
5. **Conditional Waypoints**: "Wait for sensor input" logic
6. **Demo Chaining**: Play sequence of demos
7. **Flash Wear Stats**: Track erase counts, warn before endurance limit

### Additional Storage Users
- **Calibration Data**: Joint offsets, servo trim
- **User Preferences**: Speed limits, safety zones
- **Trajectory Library**: Pre-computed IK solutions
- **Event Logs**: Record errors, crashes for debugging

---

## File Structure

```
app/
├── src/
│   ├── storage/
│   │   ├── demo_storage.c       ← NEW: Main storage implementation
│   │   └── demo_storage.h       ← NEW: Public API
│   ├── hal/
│   │   ├── hal_flash.c          ← NEW: Flash HAL wrapper
│   │   └── hal_flash.h          ← NEW: Flash HAL API
│   └── comms/
│       └── host_comms.c         ← UPDATE: Integrate flash calls
├── include/
│   ├── demo_storage.h           ← NEW: Public header
│   └── hal_flash.h              ← NEW: HAL header
└── CMakeLists.txt               ← UPDATE: Add storage sources
```

---

## Summary

### Recommended Approach: NVS-Based Storage

**Why NVS?**
- Automatic wear leveling (extends flash life)
- CRC validation built-in
- Simple key-value API
- Battle-tested in Zephyr
- ~1 KB overhead acceptable for 192 KB partition

**Why Not Alternatives?**
- Raw flash: Manual sector management, no wear leveling
- Flash Stream: Not suitable for random access (play any demo)
- File System (LittleFS): Overkill for 3 fixed slots, higher overhead

### Implementation Priority
1. **Phase 1** (Immediate): Flash HAL wrapper + demo_storage core API
2. **Phase 2** (Same sprint): Save/load integration in host_comms
3. **Phase 3** (Validation): Unit tests + integration tests
4. **Phase 4** (Polish): Error handling, statistics, recovery

### Key Decisions Made
- ✅ Storage size: 16 KB per demo (generous headroom)
- ✅ Backend: NVS for reliability
- ✅ Structure: Fixed 3-slot layout (simple, predictable)
- ✅ Integrity: CRC32 + magic number validation
- ✅ API: Blocking (sufficient for infrequent operations)

**Ready for implementation when you approve this plan!**
