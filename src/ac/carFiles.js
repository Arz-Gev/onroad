// A car's data files as { lower-case name: text }: fetched in the browser as one JSON pack (written by
// tools/ac/pack.mjs), or read from an AC car's data folder in Node (tools/ac/files.mjs).

export async function fetchCarFiles(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url}: ${res.status}`);
  return lowerKeys(await res.json());
}

export function lowerKeys(files) {
  const out = {};
  for (const [k, v] of Object.entries(files)) out[k.toLowerCase()] = v;
  return out;
}
