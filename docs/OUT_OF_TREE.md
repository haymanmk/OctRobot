# Zephyr Out-of-Tree Application Guide

## What is Out-of-Tree?

An **out-of-tree application** is a Zephyr project that lives in its own Git repository, separate from the Zephyr kernel source tree. The Zephyr kernel and its modules are pulled in as dependencies using West.

## OctroBot Uses Out-of-Tree (Correct Choice ✅)

### Your Current Structure

```
/home/hayman/Workspace/octrobot/          ← Your Git repo (manifest repo)
├── west.yml                              ← References Zephyr as dependency
├── app/                                  ← Your application code
├── boards/                               ← Your custom boards
└── .west/                                ← Created by `west init -l .`
    └── modules/
        └── zephyr/                       ← Zephyr pulled here by `west update`
            ├── kernel/
            ├── drivers/
            ├── boards/
            └── ...
```

### Alternative: In-Tree (NOT for You)

In-tree applications live inside the Zephyr source tree:

```
/opt/zephyrproject/                       ← Zephyr workspace root
├── zephyr/                               ← Zephyr kernel Git repo
│   ├── samples/
│   │   └── hello_world/                  ← In-tree sample
│   ├── boards/
│   ├── kernel/
│   └── ...
└── modules/                              ← External modules
```

**When to use in-tree:**
- Contributing to Zephyr kernel development
- Creating official Zephyr samples
- Testing kernel changes

**NOT suitable for:**
- Production applications (like OctroBot)
- Projects with independent versioning
- Custom hardware (requires custom boards)

## Why Out-of-Tree for OctroBot?

### ✅ Advantages

1. **Independent Version Control**
   - Your firmware has its own Git history
   - Zephyr updates are explicit via `west.yml`
   - No conflicts with Zephyr kernel commits

2. **Custom Board Support**
   - M5Stack Atom Lite board definition stays with your code
   - Not dependent on upstream Zephyr accepting your board
   - Iterate on board config without Zephyr PRs

3. **Reproducible Builds**
   - Pin Zephyr to specific version (v3.6.0 in your case)
   - Control when to upgrade Zephyr
   - CI/CD gets consistent environment

4. **Clean Separation**
   - Application code separate from RTOS internals
   - Easy to onboard new developers (just clone octrobot repo)
   - Clearer project ownership

5. **Production Ready**
   - Standard practice for commercial products
   - Easier to maintain across product lifecycle
   - Can vendor Zephyr if needed (copy to local Git)

### ❌ Disadvantages (Minor)

1. **Initial Setup Complexity**
   - Need to understand West manifests
   - One extra step: `west update` before first build
   - *(Very minor - worth it for production)*

2. **Disk Space**
   - Zephyr downloaded separately per workspace
   - *(Can share Zephyr across projects with West workspaces)*

## West Manifest Explained

Your `west.yml`:

```yaml
manifest:
  version: "0.13"
  
  self:
    path: octrobot              # This repo is the manifest repo
  
  projects:
    - name: zephyr
      remote: zephyrproject-rtos
      revision: v3.6.0          # Pin to specific Zephyr release
      import: true              # Import Zephyr's dependencies too
```

- **`self`**: Declares octrobot as the manifest repository (meta-repo)
- **`projects`**: Lists Zephyr and other dependencies
- **`revision`**: Locks Zephyr to v3.6.0 (can update later)
- **`import: true`**: Automatically imports Zephyr's own dependencies (HAL libraries, etc.)

## Workflow

### Initial Setup (Once)

```bash
cd /home/hayman/Workspace/octrobot
west init -l .                  # Initialize octrobot as manifest repo
west update                     # Download Zephyr and dependencies
```

### Daily Development

```bash
west build -b m5stack_atom_lite app
west flash
west espressif monitor
```

### Upgrading Zephyr

Edit `west.yml`, change `revision: v3.6.0` to `revision: v3.7.0`, then:

```bash
west update                     # Pull new Zephyr version
west build -b m5stack_atom_lite app --pristine
```

## Common Out-of-Tree Patterns

### 1. Single Application (Your Current Setup)

```
octrobot/
├── west.yml
├── app/                        # Single application
└── boards/
```

### 2. Multiple Applications

```
octrobot/
├── west.yml
├── firmware/                   # Main robot firmware
├── bootloader/                 # Custom bootloader
├── test_harness/               # Hardware test fixture
└── boards/
```

Build specific app: `west build -b m5stack_atom_lite bootloader/`

### 3. With Custom Modules

```
octrobot/
├── west.yml                    # References custom modules
├── app/
├── boards/
└── modules/
    └── octrobot_drivers/       # Reusable driver library
        ├── zephyr/module.yml   # Zephyr module metadata
        └── drivers/
```

## Comparison Table

| Aspect | Out-of-Tree (OctroBot) | In-Tree |
|--------|------------------------|---------|
| **Git Repo** | Separate (octrobot.git) | Inside Zephyr |
| **Custom Boards** | In your repo | PR to Zephyr |
| **Zephyr Version** | You control | Use what's checked out |
| **Build Command** | `west build -b m5stack_atom_lite app` | `west build -b m5stack_atom_lite` |
| **Use Case** | Production apps | Samples, testing |
| **Zephyr Updates** | Explicit in west.yml | `git pull` |
| **Onboarding** | Clone + west update | Clone entire Zephyr |

## Best Practices

1. **Pin Zephyr Version**: Use specific tags (v3.6.0) not branches (main)
2. **Test Before Upgrading**: Validate new Zephyr versions in a branch first
3. **Document Build**: Keep `README.md` and `QUICKSTART.md` updated
4. **Version Control west.yml**: Commit changes to manifest
5. **CI/CD**: Use `west update` in CI to ensure reproducible builds
6. **Custom Boards**: Keep board definitions in your repo, not personal forks

## Resources

- [Zephyr Application Development](https://docs.zephyrproject.org/latest/develop/application/index.html)
- [West Manifests](https://docs.zephyrproject.org/latest/develop/west/manifest.html)
- [Out-of-Tree Boards](https://docs.zephyrproject.org/latest/hardware/porting/board_porting.html#out-of-tree-board-definitions)

---

**You're on the right path!** Out-of-tree is the industry standard for production Zephyr applications.
