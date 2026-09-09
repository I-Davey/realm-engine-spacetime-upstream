import { expect, it, vi } from 'vitest';
import { MovementController } from '../MovementController.js';
import { sendDllFeature } from '../../../../bridge/DllFeatureBus.js';
import type { BridgeDeps } from '../../BridgeDeps.js';
vi.mock('../../../../bridge/DllFeatureBus.js', () => ({ sendDllFeature: vi.fn(() => true) }));
it('selects the experimental mode without changing existing native mode IDs', () => {
  const movement = new MovementController({} as BridgeDeps);
  for (const [mode, index] of [['unified', 7], ['spacetime', 8]] as const) {
    expect(movement.setMode(mode)).toBe(true);
    expect(sendDllFeature).toHaveBeenLastCalledWith('autoDodgeMode', index);
  }
});
it('clears native waypoints even with no script target cached', () => {
  const movement = new MovementController({ clientRef: { current: { connected: true } } } as unknown as BridgeDeps);
  movement.clearWaypoint();
  expect(sendDllFeature).toHaveBeenCalledWith('walkTargetActive', false);
  movement.navigateTo(10, 20); movement.clearWaypoint();
  expect(movement.getTarget()).toBeNull();
  expect(sendDllFeature).toHaveBeenLastCalledWith('walkTargetActive', false);
});
