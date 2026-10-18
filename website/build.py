"""Build the dependency-free ManiMux landing page for static hosting."""
from html.parser import HTMLParser
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent


class PageLinks(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()
        self.links = []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if attrs.get('id'):
            if attrs['id'] in self.ids:
                raise ValueError(f"Duplicate HTML id: {attrs['id']}")
            self.ids.add(attrs['id'])
        for key in ('src', 'href', 'poster', 'data-src'):
            if attrs.get(key):
                self.links.append(attrs[key])


if __name__ == '__main__':
    page = PageLinks()
    page.feed((ROOT / 'index.html').read_text())
    for link in page.links:
        if link.startswith('#'):
            if link[1:] and link[1:] not in page.ids:
                raise ValueError(f'Missing section: {link}')
        elif not link.startswith(('https:', 'http:', 'mailto:')):
            if not (ROOT / link).is_file():
                raise ValueError(f'Missing asset: {link}')
    destination = ROOT / 'dist'
    destination.mkdir(exist_ok=True)
    for name in ('index.html', 'styles.css', 'app.js'):
        shutil.copy2(ROOT / name, destination / name)
    shutil.copytree(ROOT / 'assets', destination / 'assets', dirs_exist_ok=True)
    print(f'Validated {len(page.links)} asset/anchor references. Built {destination}')
