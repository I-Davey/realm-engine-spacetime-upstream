import { movementKeyOptions } from './auto-dodge/keyOptions.js';
import type { PluginContext } from './api.js';
import { sendDllFeature } from './api.js';

export function register(ctx: PluginContext) {
  ctx.name = 'Target Assist'; ctx.category = 'combat';
  const push = () => {
    sendDllFeature('targetAssistSelectKey', ctx.getSetting<string>('selectKey'));
    sendDllFeature('targetAssistClearKey', ctx.getSetting<string>('clearKey'));
    sendDllFeature('targetAssistRangeFactor', ctx.getSetting<number>('rangeSafety'));
    sendDllFeature('targetAssistDebug', ctx.getSetting<boolean>('debugOverlay'));
    sendDllFeature('targetAssistEnabled', ctx.enabled);
  };
  ctx.registerSetting('rangeSafety', { label: 'Weapon range factor', type: 'range', value: .92,
    min: .5, max: 1, step: .01,
    description: 'With Spacetime selected, approach any clear position inside this fraction of the weapon range.' }, push);
  ctx.registerSetting('debugOverlay', { label: 'Show target details', type: 'boolean', value: false }, push);
  ctx.registerSetting('selectKey', {
    label: 'Select / toggle target under cursor', type: 'select', value: 'MOUSE3', options: movementKeyOptions,
    description: 'Works in the game. Hold-to-override is configured in Auto Dodge.',
  }, push);
  ctx.registerSetting('clearKey', {
    label: 'Clear target key', type: 'select', value: 'ESCAPE', options: movementKeyOptions,
  }, push);
  ctx.registerSetting('clear', { label: 'Clear selected target', type: 'button', value: null },
    () => sendDllFeature('targetAssistClear', true));
  ctx.onEnabledChange(push); ctx.on('clientConnected', push);
  ctx.registerCleanup(() => sendDllFeature('targetAssistEnabled', false));
}
