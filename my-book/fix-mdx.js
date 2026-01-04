import fs from "fs";
import path from "path";

const ROOT = "docs";

function walk(dir) {
  for (const file of fs.readdirSync(dir)) {
    const full = path.join(dir, file);
    if (fs.statSync(full).isDirectory()) walk(full);
    else if (full.endsWith(".md") || full.endsWith(".mdx")) fixFile(full);
  }
}

function fixFile(file) {
  const lines = fs.readFileSync(file, "utf8").split("\n");
  let out = [];
  let changed = false;

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];

    const looksLikeCode =
      /^[ \t]*(if|assert|for|while|def|return|[a-zA-Z_]+\s*=)/.test(line) &&
      /[<>]=?|==/.test(line);

    if (looksLikeCode) {
      out.push("```python");
      out.push(line);
      out.push("```");
      changed = true;
    } else {
      out.push(line);
    }
  }

  if (changed) {
    fs.writeFileSync(file, out.join("\n"));
    console.log("Fixed:", file);
  }
}

walk(ROOT);
