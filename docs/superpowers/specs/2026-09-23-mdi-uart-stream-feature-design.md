# MDI UART Stream Feature Design

## Scope

Migrate only STM32G431 and `peripheral_template` to a reusable UART Stream
feature. AT32 and CH592 legacy `port_mdi` implementations remain unchanged.
No build or runtime validation is part of this change while SMO work is in
progress.

## Goals

- Keep UART queueing and frame timing in one reusable MDI feature.
- Let one feature bind any number of UART resources without duplicating
  `Read`, `Write`, `Available`, `IsBusy`, Tick, or IRQ logic.
- Keep chip-specific code limited to UART register primitives and startup
  initialization.
- Preserve the static MDI call path: no device lookup, function table, or
  runtime UART object.
- Keep application calls unchanged: `MDI_STREAM_Read/Write(board_stream, ...)`.

## Design

### Common feature

Add `modus/src/mdi/feature/uart_stream.h`. It owns a typed stream state with:

- TX and RX `mringbuf_t` queues and their storage;
- TX busy state;
- RX idle-frame state and millisecond guard counter;
- compile-time binding macros that generate stream operations, `Tick_1MS`,
  and `IRQ` functions for each resource.

The feature receives backend primitive macros/functions as binding parameters.
Those primitives operate on a compile-time UART expression and do not expose a
runtime callback table.

### G431 backend

`peripheral/stm32g431/mdi/backend.h` keeps only STM32G4-specific UART work:

- USART register status and data access;
- TX complete clear and TX interrupt enable/disable;
- USART2 GPIO, clock, baud rate, and NVIC initialization;
- a small `MDI_G431_UART_STREAM_BIND` wrapper that connects STM32 primitives
  to the common feature.

The backend does not implement stream queueing or frame timing.

### Instance and lifecycle

`peripheral/stm32g431/mdi/instance.h` declares the state and binds:

```c
MDI_G431_UART_STREAM_BIND(board_stream, g_tG431Stream, USART2)
```

`port_sys.c` initializes the bound UART. `mdi/service.c` calls the generated
`board_stream` Tick function from `mdi_Clock()`. The G431 interrupt adapter
calls the generated `board_stream` IRQ function from `USART2_IRQHandler()`.

### Template

`peripheral_template/mdi/backend.h` receives a small mock UART register model
and uses the same common feature binding. The template demonstrates that
multiple UART resources can be added by state plus binding only; feature code
is not copied.

## Semantics

- `Write` appends until the TX queue is full and returns the number accepted.
- The IRQ sends one queued byte per TX-complete event and disables TX IRQ when
  the queue is empty.
- RX IRQ appends one received byte and restarts the idle-frame guard.
- `Tick_1MS` advances the guard; `Read` returns data only after the guard
  reaches the ready state, matching the current G431 and reference behavior.
- `Available` reports queued RX bytes; `IsBusy` reports TX activity.
- Null state/data and lengths beyond the feature's bounded integer interface
  return an error without touching hardware.

## Verification boundary

Before building, perform static checks only:

1. Search that G431 no longer references `halusart` or duplicate stream logic.
2. Confirm every generated resource has exactly one instance binding, one
   `mdi_Clock` Tick call, and one IRQ entry.
3. Run `git diff --check`.

Build and target execution remain deferred until the user finishes SMO work.
