# AI Runtime Instructions

Runtime Instructions are **allowlisted** actions applied to the live game world. They are separate from dev tasks and are safe to apply at runtime.

## Schema (JSON)
Top-level:
- Either an array of instructions **or**
- An object with `instructions: [...]`

Each instruction:
- `action`: one of:
  - `spawn_entity`
  - `set_transform`
  - `set_component`
  - `trigger_dialogue`
  - `load_level`
  - `schedule_event`
- Additional fields depend on the action.

### Example
```
{
  "instructions": [
    {
      "action": "spawn_entity",
      "name": "runtime_npc",
      "prefab": "demo_npc",
      "transform": {
        "position": [1.0, 0.0, 0.0],
        "rotation": [0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0]
      }
    },
    {
      "action": "set_transform",
      "name": "runtime_npc",
      "transform": {
        "position": [2.0, 0.0, 0.0]
      }
    }
  ]
}
```

## Behavior Today
- `spawn_entity` and `set_transform` are implemented in `minimal_app`.
- `set_component`, `trigger_dialogue`, `load_level`, `schedule_event` are stubs that log.

## Safety
Only allowlisted actions are executed. Unknown actions are ignored.
