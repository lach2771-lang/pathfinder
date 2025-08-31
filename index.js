// A* Pathfinder for ChatTriggers v2 (MC 1.8.9) - FIXED VERSION
// Converted from working Forge mod pathfinder
// Usage: /astar <x> <y> <z> to pathfind to coordinates

// Configuration
const CONFIG = {
    maxDistance: 100,
    arrivalDistance: 2.0,
    sneakDistance: 5.0,
    sprintDistance: 10.0,
    rotationSpeed: 4.5,
    minYawDiff: 5.0,
    maxIterations: 50000,
    debug: false
}

// Path Node class
class PathNode {
    constructor(pos, parent, gCost, hCost) {
        this.pos = pos  // {x, y, z}
        this.parent = parent
        this.gCost = gCost
        this.hCost = hCost
        this.fCost = gCost + hCost
    }
}

// Main Pathfinder class
class ChatTriggersPathfinder {
    constructor() {
        this.active = false
        this.target = null
        this.currentPath = null
        this.currentPathIndex = 0
        this.lastNodeChangeTime = 0
        this.reachedTarget = false
        
        this.registerCommands()
        this.startTickLoop()
    }

    // A* pathfinding algorithm
    findPath(start, goal) {
        if (this.posEquals(start, goal)) {
            return [start]
        }

        const openSet = []
        const closedSet = new Set()
        const allNodes = new Map()

        const startNode = new PathNode(start, null, 0, this.heuristic(start, goal))
        openSet.push(startNode)
        allNodes.set(this.posKey(start), startNode)

        let iterations = 0
        while (openSet.length > 0 && iterations < CONFIG.maxIterations) {
            iterations++
            
            // Sort by fCost and get lowest
            openSet.sort((a, b) => a.fCost - b.fCost)
            const current = openSet.shift()

            if (this.posEquals(current.pos, goal)) {
                return this.reconstructPath(current)
            }

            closedSet.add(this.posKey(current.pos))

            for (const neighbor of this.getNeighbors(current.pos)) {
                const neighborKey = this.posKey(neighbor)
                
                if (closedSet.has(neighborKey) || !this.isWalkable(neighbor)) {
                    continue
                }

                const tentativeGCost = current.gCost + this.getDistance(current.pos, neighbor)
                let neighborNode = allNodes.get(neighborKey)

                if (!neighborNode) {
                    neighborNode = new PathNode(neighbor, current, tentativeGCost, this.heuristic(neighbor, goal))
                    allNodes.set(neighborKey, neighborNode)
                    openSet.push(neighborNode)
                } else if (tentativeGCost < neighborNode.gCost) {
                    neighborNode.parent = current
                    neighborNode.gCost = tentativeGCost
                    neighborNode.fCost = neighborNode.gCost + neighborNode.hCost
                }
            }
        }

        return [] // No path found
    }

    // Reconstruct path from goal to start
    reconstructPath(node) {
        const path = []
        while (node) {
            path.unshift(node.pos)
            node = node.parent
        }
        return path
    }

    // Heuristic function
    heuristic(a, b) {
        const dx = a.x - b.x
        const dy = a.y - b.y
        const dz = a.z - b.z
        return Math.sqrt(dx * dx + dy * dy + dz * dz)
    }

    // Distance between two positions
    getDistance(a, b) {
        const dx = Math.abs(a.x - b.x)
        const dy = Math.abs(a.y - b.y)
        const dz = Math.abs(a.z - b.z)

        // Diagonal movement cost
        if (dx > 0 && dz > 0) {
            return 1.414 // sqrt(2)
        }
        // Vertical movement cost
        if (dy > 0) {
            return 1.2
        }
        // Horizontal movement cost
        return 1.0
    }

    // Get neighbors for pathfinding - FIXED VERSION
    getNeighbors(pos) {
        const neighbors = []
        
        // Check horizontal movements at same level
        for (let dx = -1; dx <= 1; dx++) {
            for (let dz = -1; dz <= 1; dz++) {
                if (dx === 0 && dz === 0) continue
                
                // Check if we can walk to this position at the same level
                const sameLevel = { x: pos.x + dx, y: pos.y, z: pos.z + dz }
                if (this.canMoveTo(pos, sameLevel)) {
                    neighbors.push(sameLevel)
                }
                
                // Check if we need to go up (there's a block in the way at same level)
                const upLevel = { x: pos.x + dx, y: pos.y + 1, z: pos.z + dz }
                if (!this.canMoveTo(pos, sameLevel) && this.canMoveTo(pos, upLevel)) {
                    neighbors.push(upLevel)
                }
                
                // Check if we can drop down
                const downLevel = { x: pos.x + dx, y: pos.y - 1, z: pos.z + dz }
                if (this.canDropTo(pos, downLevel)) {
                    neighbors.push(downLevel)
                }
            }
        }
        
        return neighbors
    }

    // Check if we can move from one position to another
    canMoveTo(from, to) {
        // Basic bounds check
        if (to.y < 0 || to.y > 255) return false
        
        try {
            // Check if destination is walkable
            const footBlock = World.getBlockAt(to.x, to.y, to.z)
            const headBlock = World.getBlockAt(to.x, to.y + 1, to.z)
            const groundBlock = World.getBlockAt(to.x, to.y - 1, to.z)
            
            const footClear = this.isPassableBlock(footBlock)
            const headClear = this.isPassableBlock(headBlock)
            const hasGround = this.isSolidGroundBlock(groundBlock)
            
            // If moving horizontally, check for obstruction
            if (from.y === to.y) {
                const dx = to.x - from.x
                const dz = to.z - from.z
                
                // Check if there's a clear path (no walls blocking)
                if (Math.abs(dx) === 1 && Math.abs(dz) === 1) {
                    // Diagonal movement - check both adjacent sides
                    const side1Block = World.getBlockAt(from.x + dx, from.y, from.z)
                    const side2Block = World.getBlockAt(from.x, from.y, from.z + dz)
                    const side1Clear = this.isPassableBlock(side1Block)
                    const side2Clear = this.isPassableBlock(side2Block)
                    
                    return footClear && headClear && hasGround && (side1Clear || side2Clear)
                }
            }
            
            return footClear && headClear && hasGround
        } catch (error) {
            return false
        }
    }

    // Check if we can drop down to a position
    canDropTo(from, to) {
        // Can only drop down, not up
        if (to.y >= from.y) return false
        
        // Maximum fall distance
        if (from.y - to.y > 3) return false
        
        return this.isWalkable(to)
    }

    // Simple walkability check
    isWalkable(pos) {
        try {
            // Basic bounds check
            if (pos.y < 0 || pos.y > 255) return false

            // Try to get blocks
            const footBlock = World.getBlockAt(pos.x, pos.y, pos.z)
            const headBlock = World.getBlockAt(pos.x, pos.y + 1, pos.z)
            const groundBlock = World.getBlockAt(pos.x, pos.y - 1, pos.z)

            // Simple logic: foot and head should be passable, ground should exist and be solid
            const footClear = this.isPassableBlock(footBlock)
            const headClear = this.isPassableBlock(headBlock)
            const hasGround = this.isSolidGroundBlock(groundBlock)

            return footClear && headClear && hasGround
        } catch (error) {
            // If block checking fails, assume not walkable
            return false
        }
    }

    // Check if block is passable (robust for ChatTriggers API variants)
    isPassableBlock(block) {
        if (!block) return true
        try {
            const blockType = block.type || block
            if (blockType && typeof blockType.getID === "function") {
                return blockType.getID() === 0
            }
            if (typeof block.getID === "function") {
                return block.getID() === 0
            }
            const name = (typeof blockType.getRegistryName === "function"
                ? blockType.getRegistryName()
                : (typeof blockType.getName === "function" ? blockType.getName() : (blockType.name || "")))
                .toString()
                .toLowerCase()
            return name.includes("air")
        } catch (error) {
            // If unsure, assume passable to avoid over-blocking
            return true
        }
    }

    // Check if block is a liquid (not solid ground)
    isLiquidBlock(block) {
        if (!block) return false
        try {
            const blockType = block.type || block
            const name = (typeof blockType.getRegistryName === "function"
                ? blockType.getRegistryName()
                : (typeof blockType.getName === "function" ? blockType.getName() : (blockType.name || "")))
                .toString()
                .toLowerCase()
            return name.includes("water") || name.includes("lava")
        } catch (error) {
            return false
        }
    }

    // Determine if a block can serve as solid ground support
    isSolidGroundBlock(block) {
        return !!block && !this.isPassableBlock(block) && !this.isLiquidBlock(block)
    }

    // Position helper functions
    posEquals(a, b) {
        return Math.floor(a.x) === Math.floor(b.x) && 
               Math.floor(a.y) === Math.floor(b.y) && 
               Math.floor(a.z) === Math.floor(b.z)
    }

    posKey(pos) {
        return `${Math.floor(pos.x)},${Math.floor(pos.y)},${Math.floor(pos.z)}`
    }

    // Resolve a target position to a nearby walkable block space
    resolveGoalToWalkable(goalPos) {
        const base = {
            x: Math.floor(goalPos.x),
            y: Math.floor(goalPos.y),
            z: Math.floor(goalPos.z)
        }

        if (this.isWalkable(base)) return base

        const verticalOffsets = [1, 2, -1, -2, 3, -3]
        for (const dy of verticalOffsets) {
            const candidate = { x: base.x, y: base.y + dy, z: base.z }
            if (this.isWalkable(candidate)) return candidate
        }

        const maxRadius = 2
        for (let radius = 1; radius <= maxRadius; radius++) {
            for (let dx = -radius; dx <= radius; dx++) {
                for (let dz = -radius; dz <= radius; dz++) {
                    if (dx === 0 && dz === 0) continue
                    const candidateBase = { x: base.x + dx, y: base.y, z: base.z + dz }
                    if (this.isWalkable(candidateBase)) return candidateBase
                    for (const dy of verticalOffsets) {
                        const candidate = { x: candidateBase.x, y: candidateBase.y + dy, z: candidateBase.z }
                        if (this.isWalkable(candidate)) return candidate
                    }
                }
            }
        }

        return null
    }

    // Start pathfinding to target
    startPathfinding(targetVec) {
        this.target = targetVec
        this.active = true
        this.reachedTarget = false
        this.currentPath = null
        this.currentPathIndex = 0

        if (CONFIG.debug) {
            ChatLib.chat(`&aStarted pathfinding to ${targetVec.x.toFixed(1)}, ${targetVec.y.toFixed(1)}, ${targetVec.z.toFixed(1)}`)
        }
    }

    // Stop pathfinding
    stopPathfinding() {
        this.active = false
        this.target = null
        this.currentPath = null
        this.currentPathIndex = 0
        this.reachedTarget = false
        this.stopMovement()

        if (CONFIG.debug) {
            ChatLib.chat("&ePathfinding stopped")
        }
    }

    // Stop all movement
    stopMovement() {
        try {
            // Release all movement keys
            const gameSettings = Client.getMinecraft().field_71474_y
            gameSettings.field_74351_w.field_74513_e = false // forward
            gameSettings.field_74368_y.field_74513_e = false // back
            gameSettings.field_74370_x.field_74513_e = false // left
            gameSettings.field_74366_z.field_74513_e = false // right
            gameSettings.field_74311_E.field_74513_e = false // sneak
            gameSettings.field_74308_b.field_74513_e = false // sprint
            gameSettings.field_74314_A.field_74513_e = false // jump
        } catch (error) {
            // Ignore key binding errors
        }
    }

    // Main pathfinding execution
    executePathfinding() {
        const player = Player.getPlayer()
        const rawStartPos = {
            x: Math.floor(player.field_70165_t), // posX - use floor for consistency
            y: Math.floor(player.field_70163_u), // posY  
            z: Math.floor(player.field_70161_v)  // posZ
        }

        const rawGoalPos = {
            x: Math.floor(this.target.x),
            y: Math.floor(this.target.y),
            z: Math.floor(this.target.z)
        }

        // Check if we've reached the target
        const distanceToTarget = this.heuristic(rawStartPos, this.target)
        if (distanceToTarget < CONFIG.arrivalDistance) {
            if (CONFIG.debug) {
                ChatLib.chat("&aReached target!")
            }
            this.stopPathfinding()
            return
        }

        // Generate new path if needed
        if (!this.currentPath || this.currentPathIndex >= this.currentPath.length) {
            if (CONFIG.debug) {
                ChatLib.chat(`&7Generating path from ${rawStartPos.x},${rawStartPos.y},${rawStartPos.z} to ${rawGoalPos.x},${rawGoalPos.y},${rawGoalPos.z}`)
            }

            const resolvedStart = this.resolveGoalToWalkable(rawStartPos)
            if (!resolvedStart) {
                ChatLib.chat("&cYour current position is not in or near a walkable space")
                this.stopPathfinding()
                return
            }
            const playerPos = resolvedStart

            const resolvedGoal = this.resolveGoalToWalkable(rawGoalPos)
            if (!resolvedGoal) {
                ChatLib.chat("&cTarget is not reachable: no nearby walkable space")
                this.stopPathfinding()
                return
            }

            if (CONFIG.debug && (resolvedGoal.x !== rawGoalPos.x || resolvedGoal.y !== rawGoalPos.y || resolvedGoal.z !== rawGoalPos.z)) {
                ChatLib.chat(`&7Resolved goal to ${resolvedGoal.x},${resolvedGoal.y},${resolvedGoal.z}`)
            }

            this.currentPath = this.findPath(playerPos, resolvedGoal)
            this.currentPathIndex = 0

            if (!this.currentPath || this.currentPath.length === 0) {
                ChatLib.chat("&cNo path found!")
                this.stopPathfinding()
                return
            }

            if (CONFIG.debug) {
                ChatLib.chat(`&aPath generated: ${this.currentPath.length} nodes`)
            }
        }

        // Execute movement
        this.executeMovement()
    }

    // Execute movement along path
    executeMovement() {
        if (!this.currentPath || this.currentPathIndex >= this.currentPath.length) return

        const player = Player.getPlayer()
        const playerPos = {
            x: player.field_70165_t,
            y: player.field_70163_u,
            z: player.field_70161_v
        }

        const currentTarget = this.currentPath[this.currentPathIndex]
        const distanceToNode = this.heuristic(playerPos, currentTarget)

        // Check if we've reached current path node
        if (distanceToNode < 2.0) {
            this.currentPathIndex++
            this.lastNodeChangeTime = Date.now()

            if (CONFIG.debug) {
                ChatLib.chat(`&7Advanced to node ${this.currentPathIndex}/${this.currentPath.length}`)
            }

            if (this.currentPathIndex >= this.currentPath.length) {
                this.currentPath = null
                return
            }
        }

        const targetVec = {
            x: currentTarget.x + 0.5,
            y: currentTarget.y,
            z: currentTarget.z + 0.5
        }

        // Calculate target yaw
        const dx = targetVec.x - playerPos.x
        const dz = targetVec.z - playerPos.z
        const targetYaw = Math.atan2(-dx, dz) * 180 / Math.PI

        // Smooth rotation
        const currentYaw = player.field_70177_z // rotationYaw
        let yawDiff = targetYaw - currentYaw

        // Normalize angle
        while (yawDiff > 180) yawDiff -= 360
        while (yawDiff < -180) yawDiff += 360

        if (Math.abs(yawDiff) > CONFIG.minYawDiff) {
            let yawChange = yawDiff
            if (Math.abs(yawChange) > CONFIG.rotationSpeed) {
                yawChange = Math.sign(yawChange) * CONFIG.rotationSpeed
            }
            player.field_70177_z = currentYaw + yawChange
        }

        // Determine movement direction
        const horizontalDistance = Math.sqrt(dx * dx + dz * dz)
        const verticalDistance = targetVec.y - playerPos.y

        if (horizontalDistance > 0.1) {
            // Calculate which keys to press based on angle
            const angleRad = Math.atan2(-dx, dz)
            const angleDeg = angleRad * 180 / Math.PI
            const playerYaw = player.field_70177_z

            // Normalize the relative angle
            let relativeAngle = angleDeg - playerYaw
            while (relativeAngle > 180) relativeAngle -= 360
            while (relativeAngle < -180) relativeAngle += 360

            // Determine movement keys
            const forward = Math.abs(relativeAngle) < 45
            const backward = Math.abs(relativeAngle) > 135
            const left = relativeAngle > 45 && relativeAngle < 135
            const right = relativeAngle < -45 && relativeAngle > -135

            // Apply movement
            this.setMovementKey("forward", forward && !backward)
            this.setMovementKey("back", backward && !forward)
            this.setMovementKey("left", left && !forward && !backward)
            this.setMovementKey("right", right && !forward && !backward)

            // Speed control
            const distanceToFinalTarget = this.heuristic(playerPos, this.target)
            const shouldSneak = distanceToFinalTarget < CONFIG.sneakDistance
            const shouldSprint = !shouldSneak && distanceToFinalTarget > CONFIG.sprintDistance && Math.abs(yawDiff) < 40

            this.setMovementKey("sneak", shouldSneak)
            this.setMovementKey("sprint", shouldSprint)
        }

        // Jump logic
        const shouldJump = this.shouldJumpToReachTarget(playerPos, targetVec)
        this.setMovementKey("jump", shouldJump && player.field_70122_E) // onGround
    }

    // Simple jump logic
    shouldJumpToReachTarget(playerPos, targetPos) {
        const horizontalDistance = Math.sqrt(
            Math.pow(targetPos.x - playerPos.x, 2) + 
            Math.pow(targetPos.z - playerPos.z, 2)
        )
        const verticalDistance = targetPos.y - playerPos.y

        return verticalDistance > 0.5 && horizontalDistance < 2.0
    }

    // Set movement key state
    setMovementKey(key, pressed) {
        try {
            const gameSettings = Client.getMinecraft().field_71474_y
            let keyBind

            switch (key) {
                case "forward":
                    keyBind = gameSettings.field_74351_w // keyBindForward
                    break
                case "back":
                    keyBind = gameSettings.field_74368_y // keyBindBack
                    break
                case "left":
                    keyBind = gameSettings.field_74370_x // keyBindLeft
                    break
                case "right":
                    keyBind = gameSettings.field_74366_z // keyBindRight
                    break
                case "jump":
                    keyBind = gameSettings.field_74314_A // keyBindJump
                    break
                case "sneak":
                    keyBind = gameSettings.field_74311_E // keyBindSneak
                    break
                case "sprint":
                    keyBind = gameSettings.field_74308_b // keyBindSprint
                    break
            }

            if (keyBind) {
                keyBind.field_74513_e = pressed // pressed
            }
        } catch (error) {
            // Ignore key binding errors
        }
    }

    // Position utility functions
    posEquals(a, b) {
        return Math.floor(a.x) === Math.floor(b.x) &&
               Math.floor(a.y) === Math.floor(b.y) &&
               Math.floor(a.z) === Math.floor(b.z)
    }

    posKey(pos) {
        return `${Math.floor(pos.x)},${Math.floor(pos.y)},${Math.floor(pos.z)}`
    }

    // Main tick function
    onTick() {
        if (!this.active || !this.target) return

        try {
            this.executePathfinding()
        } catch (error) {
            if (CONFIG.debug) {
                ChatLib.chat(`&cPathfinding error: ${error.message}`)
            }
            this.stopPathfinding()
        }
    }

    // Start the tick loop
    startTickLoop() {
        register("tick", () => {
            this.onTick()
        })
    }

    // Register commands
    registerCommands() {
        const self = this

        register("command", function(x, y, z) {
            if (!x || !y || !z) {
                ChatLib.chat("&cUsage: /astar <x> <y> <z>")
                return
            }

            const targetX = parseFloat(x)
            const targetY = parseFloat(y) 
            const targetZ = parseFloat(z)

            if (isNaN(targetX) || isNaN(targetY) || isNaN(targetZ)) {
                ChatLib.chat("&cInvalid coordinates!")
                return
            }

            const player = Player.getPlayer()
            const distance = Math.sqrt(
                Math.pow(targetX - player.field_70165_t, 2) +
                Math.pow(targetY - player.field_70163_u, 2) +
                Math.pow(targetZ - player.field_70161_v, 2)
            )

            if (distance > CONFIG.maxDistance) {
                ChatLib.chat(`&cTarget too far! Distance: ${distance.toFixed(1)}, Max: ${CONFIG.maxDistance}`)
                return
            }

            self.startPathfinding({ x: targetX, y: targetY, z: targetZ })
            ChatLib.chat(`&aPathfinding to ${targetX}, ${targetY}, ${targetZ} (distance: ${distance.toFixed(1)})`)
        }).setName("astar")

        register("command", function() {
            self.stopPathfinding()
            ChatLib.chat("&ePathfinding stopped")
        }).setName("astop")

        register("command", function() {
            CONFIG.debug = !CONFIG.debug
            ChatLib.chat(`&aDebug mode: ${CONFIG.debug ? "ON" : "OFF"}`)
        }).setName("adebug")

        register("command", function() {
            if (self.active) {
                const progress = self.currentPathIndex && self.currentPath 
                    ? Math.round((self.currentPathIndex / self.currentPath.length) * 100) 
                    : 0
                ChatLib.chat(`&aPathfinding active: ${progress}% complete`)
                if (CONFIG.debug && self.currentPath) {
                    ChatLib.chat(`&7Node ${self.currentPathIndex}/${self.currentPath.length}`)
                }
            } else {
                ChatLib.chat("&eNot currently pathfinding")
            }
        }).setName("astatus")
    }
}

// Initialize the pathfinder
const pathfinder = new ChatTriggersPathfinder()

ChatLib.chat("&aA* Pathfinder loaded!")
ChatLib.chat("&7Commands: /astar <x> <y> <z>, /astop, /adebug, /astatus")