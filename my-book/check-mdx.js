const fs = require('fs');
const path = require('path');

const dir = path.join(__dirname, 'docs/textbook');

function walkDir(currentPath) {
  const files = fs.readdirSync(currentPath);
  files.forEach(file => {
    const fullPath = path.join(currentPath, file);
    const stat = fs.statSync(fullPath);
    if (stat.isDirectory()) {
      walkDir(fullPath);
    } else if (file.endsWith('.md') || file.endsWith('.mdx')) {
      checkFile(fullPath);
    }
  });
}

function checkFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const lines = content.split(/\r?\n/);

  let inMath = false;
  let lineNum = 0;

  lines.forEach(line => {
    lineNum++;
    if (line.includes('$$')) {
      inMath = !inMath;
      if (line.trim() === '$$') return;
    }
    if (!inMath) {
      if (line.match(/<[^>]+>/) && !line.match(/^<Equation/) ) {
        console.log(`Potential JSX/HTML wrapper in ${filePath} line ${lineNum}: ${line.trim()}`);
      }
      if (line.includes('<') || line.includes('>')) {
        console.log(`Potential raw < or > in ${filePath} line ${lineNum}: ${line.trim()}`);
      }
    }
  });

  if (inMath) {
    console.log(`Unclosed $$ math block in ${filePath}`);
  }
}

walkDir(dir);
console.log('MDX scan completed.');
