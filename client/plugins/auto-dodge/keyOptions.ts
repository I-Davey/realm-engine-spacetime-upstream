// Single physical keys handled inside the game, including while the dashboard is unfocused.
export const movementKeyOptions = [
  { label: 'Unbound', value: 'NONE' },
  ...['SHIFT', 'CTRL', 'ALT', 'SPACE', 'TAB', 'ESCAPE', 'MOUSE3', 'MOUSE4', 'MOUSE5',
    ...'ABCDEFGHIJKLMNOPQRSTUVWXYZ', ...'0123456789',
    ...Array.from({ length: 12 }, (_, i) => `F${i + 1}`)]
    .map(value => ({ label: value === 'MOUSE3' ? 'Middle mouse' : value, value })),
];
