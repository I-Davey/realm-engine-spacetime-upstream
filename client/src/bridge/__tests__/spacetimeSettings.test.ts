import { afterEach, describe, expect, it, vi } from 'vitest';
import { register } from '../../../plugins/auto-dodge.js';
import type { PluginContext } from '../../../plugins/api.js';
import { setDllFeatureSender } from '../DllFeatureBus.js';

function harness() {
  const send = vi.fn();
  setDllFeatureSender(send);
  const values = new Map<string, unknown>();
  const changes = new Map<string, (value: unknown) => void>();
  const events = new Map<string, () => void>();
  const packets = new Map<string, () => void>();
  let enabledChanged = (_enabled: boolean) => {};
  const ctx = {
    enabled: true,
    registerSetting(key: string, config: { value: unknown }, change?: (value: unknown) => void) {
      values.set(key, config.value);
      if (change) changes.set(key, change);
    },
    getSetting: (key: string) => values.get(key),
    onEnabledChange: (callback: typeof enabledChanged) => { enabledChanged = callback; },
    on: (key: string, callback: () => void) => events.set(key, callback),
    hookPacket: (key: string, callback: () => void) => packets.set(key, callback),
    hookAllPackets() {},
    registerCleanup() {},
    log() {},
  };
  register(ctx as unknown as PluginContext);
  return {
    send, events, packets, values,
    edit(key: string, value: unknown) { values.set(key, value); changes.get(key)?.(value); },
    enable(value: boolean) { ctx.enabled = value; enabledChanged(value); },
  };
}

afterEach(() => { setDllFeatureSender(null); vi.useRealTimers(); });

describe('Spacetime dashboard settings', () => {
  it('keeps independent moving and stationary defaults and restores their keys on reconnect', () => {
    const h = harness();
    expect(h.values.get('spacetimeHorizonMs')).toBe(1475);
    expect(h.values.get('spacetimeStationaryHorizonMs')).toBe(4000);
    expect(h.values.get('spacetimeLookRange')).toBe(3.5);
    expect(h.values.get('spacetimeContactScale')).toBe(1);
    expect(h.values.get('spacetimeStationaryLookRange')).toBe(10);
    h.edit('spacetimeStationaryHorizonMs', 3000);
    h.edit('spacetimeBypassKey', 'MOUSE4');
    h.edit('spacetimeOverlayKey', 'F8');
    h.edit('dodgeMode', 'spacetime');
    h.send.mockClear();
    h.events.get('clientConnected')!();
    expect(h.send).toHaveBeenCalledWith('spacetimeHorizonMs', 1475);
    expect(h.send).toHaveBeenCalledWith('spacetimeStationaryHorizonMs', 3000);
    expect(h.send).toHaveBeenCalledWith('spacetimeBypassKey', 'MOUSE4');
    expect(h.send).toHaveBeenCalledWith('spacetimeOverlayKey', 'F8');
  });
  it('sends numeric and boolean edits without rounding or truthy-string conversion', () => {
    const h = harness();
    for (const [key, value] of Object.entries({
      spacetimeLookRange: 7.5, spacetimeContactScale: 0.55,
      spacetimeHorizonMs: 4000, spacetimeMaxDistance: 12,
      spacetimeEnemyScale: 2.5, spacetimeSearchBudgetMs: 12,
    })) {
      h.edit(key, value);
      expect(h.send).toHaveBeenLastCalledWith(key, value);
    }
    h.edit('spacetimeShadowMode', false);
    expect(h.send).toHaveBeenLastCalledWith('spacetimeShadowMode', 0);
    h.edit('spacetimeAvoidBlocks', true);
    expect(h.send).toHaveBeenLastCalledWith('spacetimeAvoidBlocks', 1);
    h.edit('spacetimeAvoidBlocks', false);
    expect(h.send).toHaveBeenLastCalledWith('spacetimeAvoidBlocks', 0);
    h.edit('spacetimeDebugOverlay', true);
    expect(h.send).toHaveBeenLastCalledWith('spacetimeDebugOverlay', 1);
    h.edit('spacetimeDebugOverlay', false);
    expect(h.send).toHaveBeenLastCalledWith('spacetimeDebugOverlay', 0);
  });

  it('reapplies chosen settings before selecting or enabling the mode mid-session', () => {
    const h = harness();
    h.edit('spacetimeContactScale', 0.65);
    h.edit('spacetimeEnemyScale', 1.75);
    h.edit('spacetimeAvoidBlocks', true);
    h.edit('spacetimeSearchBudgetMs', 8);
    h.edit('spacetimeHorizonMs', 450);
    h.edit('spacetimeDebugOverlay', false);
    h.send.mockClear();
    h.edit('dodgeMode', 'spacetime');
    expect(h.send.mock.calls.findIndex(([key]) => key === 'spacetimeContactScale'))
      .toBeLessThan(h.send.mock.calls.findIndex(([key]) => key === 'autoDodgeMode'));
    h.enable(false);
    h.send.mockClear();
    h.enable(true);
    expect(h.send).toHaveBeenCalledWith('spacetimeContactScale', 0.65);
    expect(h.send).toHaveBeenCalledWith('spacetimeEnemyScale', 1.75);
    expect(h.send).toHaveBeenCalledWith('spacetimeAvoidBlocks', 1);
    expect(h.send).toHaveBeenCalledWith('spacetimeSearchBudgetMs', 8);
    expect(h.send).toHaveBeenCalledWith('spacetimeHorizonMs', 450);
    expect(h.send).toHaveBeenCalledWith('spacetimeDebugOverlay', 0);
    expect(h.send).toHaveBeenCalledWith('autoDodgeMode', 8);
  });

  it('preserves native slider overrides on portal entry but syncs on connection', () => {
    vi.useFakeTimers();
    const h = harness();
    h.edit('dodgeMode', 'spacetime');
    h.edit('spacetimeLookRange', 9);
    h.send.mockClear();
    h.packets.get('MAPINFO')!();
    vi.advanceTimersByTime(301);
    expect(h.send.mock.calls.filter(([key]) => String(key).startsWith('spacetime'))).toEqual([]);
    h.events.get('clientConnected')!();
    expect(h.send).toHaveBeenCalledWith('spacetimeLookRange', 9);
  });
});
