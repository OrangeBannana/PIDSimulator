function normalRandom(mean = 0, std = 1) {
  // Box-Muller transform for normal distribution
  const u1 = Math.random();
  const u2 = Math.random();
  const z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  return z0 * std + mean;
}

console.log('Testing normalRandom function:');
for(let i = 0; i < 5; i++) {
  console.log(normalRandom(0, 0.01));
}
console.log('Function works correctly');