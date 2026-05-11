/* =============================================
   AI NEWSFEED — app.js
   ============================================= */

const CFG = {
    CACHE_TTL:        15 * 60 * 1000,
    REFRESH_INTERVAL: 15 * 60 * 1000,
    MAX_PER_SOURCE:   10,
    IFRAME_TIMEOUT:   9000,
    CORS_PROXY:       'https://api.allorigins.win/get?url=',
};

const SOURCES = [
    { url: 'https://techcrunch.com/category/artificial-intelligence/feed/',        name: 'TechCrunch',      icon: '&#9889;' },
    { url: 'https://www.theverge.com/ai-artificial-intelligence/rss/index.xml',    name: 'The Verge',       icon: '&#9672;' },
    { url: 'https://venturebeat.com/category/ai/feed/',                             name: 'VentureBeat',     icon: '&#9650;' },
    { url: 'https://www.wired.com/feed/category/artificial-intelligence/latest/rss', name: 'Wired',         icon: '&#9679;' },
    { url: 'https://feeds.technologyreview.com/magazine/artificial-intelligence',  name: 'MIT Tech Review', icon: '&#9670;' },
];

const LLM_KW = ['llm','language model','gpt','claude','gemini','llama','mistral',
    'chatgpt','transformer','fine-tun','training data','model weights','tokens',
    'alignment','rlhf','foundation model','bert','diffusion model','prompt','inference',
    'open source model','open-source model','weights','hugging face','anthropic'];

const RETAIL_KW = ['retail','e-commerce','ecommerce','shopping','consumer',
    'amazon','store','marketplace','purchase','checkout','recommendation engine',
    'supply chain','inventory','fulfillment','personalization','shopify','walmart',
    'dynamic pricing','visual search','commerce'];

const CAT_ICONS  = { AI: '&#9674;', LLM: '&#11042;', RETAIL: '&#9632;' };

const STOPWORDS = new Set([
    'the','a','an','in','on','at','to','for','of','and','or','but','is','are',
    'was','were','will','be','been','being','has','have','had','with','as','by',
    'from','up','about','into','through','during','before','after','this','that',
    'these','those','it','its','what','which','who','how','when','where','why',
    'not','no','can','could','would','should','may','might','must','shall',
    'do','does','did','get','got','also','just','more','than','their','they',
    'them','our','your','us','we','i','me','he','she','him','her','all','says','said',
]);

let allNews      = [];
let currentFilter = 'all';
let currentItem   = null;
let iframeTimer   = null;
let refreshTimer  = null;

/* ---- Helpers ---- */

function categorize(text) {
    const t = text.toLowerCase();
    if (LLM_KW.some(k => t.includes(k)))    return 'LLM';
    if (RETAIL_KW.some(k => t.includes(k))) return 'RETAIL';
    return 'AI';
}

function threeWords(title) {
    const words = title.split(/\s+/)
        .map(w => w.replace(/[^a-zA-Z0-9]/g, ''))
        .filter(w => w.length > 2 && !STOPWORDS.has(w.toLowerCase()));
    const chosen = words.length >= 3 ? words.slice(0, 3) : title.split(/\s+/).slice(0, 3);
    return chosen.join(' ').toUpperCase();
}

function stripHtml(html) {
    const d = document.createElement('div');
    d.innerHTML = html;
    return (d.textContent || d.innerText || '').replace(/\s+/g, ' ').trim();
}

function getSentences(text, max) {
    const sentences = (text.match(/[^.!?]+[.!?]+/g) || [text])
        .map(s => s.trim()).filter(s => s.length > 20);
    return sentences.slice(0, max);
}

function timeAgo(date) {
    const m = Math.floor((Date.now() - new Date(date)) / 60000);
    if (m <  1) return 'just now';
    if (m < 60) return m + 'm ago';
    const h = Math.floor(m / 60);
    if (h < 24) return h + 'h ago';
    return Math.floor(h / 24) + 'd ago';
}

function esc(s) {
    return String(s)
        .replace(/&/g,'&amp;').replace(/</g,'&lt;')
        .replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

function getImg(item) {
    /* media:content or media:thumbnail via namespace */
    for (const tag of ['content','thumbnail']) {
        const els = item.getElementsByTagNameNS('*', tag);
        if (els.length) {
            const url = els[0].getAttribute('url');
            if (url && /\.(jpg|jpeg|png|gif|webp)/i.test(url)) return url;
        }
    }
    /* enclosure */
    const enc = item.querySelector('enclosure');
    if (enc && /^image\//i.test(enc.getAttribute('type') || '')) {
        return enc.getAttribute('url');
    }
    /* image from description HTML */
    const desc = item.querySelector('description')?.textContent || '';
    const m = desc.match(/src=["']([^"']+\.(?:jpg|jpeg|png|gif|webp)[^"']*)/i);
    if (m) return m[1];
    return null;
}

/* ---- RSS fetching ---- */

async function fetchSource(src) {
    try {
        const resp = await fetch(
            CFG.CORS_PROXY + encodeURIComponent(src.url),
            { signal: AbortSignal.timeout(14000) }
        );
        if (!resp.ok) throw new Error('HTTP ' + resp.status);
        const json = await resp.json();
        if (!json.contents) throw new Error('empty proxy response');

        const xml = new DOMParser().parseFromString(json.contents, 'application/xml');
        if (xml.querySelector('parsererror')) throw new Error('XML parse error');

        const items = Array.from(xml.querySelectorAll('item')).slice(0, CFG.MAX_PER_SOURCE);

        return items.map((item, idx) => {
            const title    = stripHtml(item.querySelector('title')?.textContent || 'Untitled');
            const link     = (item.querySelector('link')?.textContent || '#').trim();
            const descRaw  = item.querySelector('description')?.textContent  || '';
            const contRaw  = item.querySelector('encoded')?.textContent      || descRaw;
            const pubDate  = item.querySelector('pubDate')?.textContent      || '';
            const image    = getImg(item);

            const descClean = stripHtml(descRaw);
            const contClean = stripHtml(contRaw);
            const fullText  = contClean.length > descClean.length ? contClean : descClean;
            const category  = categorize(title + ' ' + descClean);

            return {
                id:        src.name + '-' + idx,
                title,
                headline:  threeWords(title),
                summary:   descClean.slice(0, 145) + (descClean.length > 145 ? '...' : ''),
                fullText,
                link,
                image,
                source:    src.name,
                category,
                date:      pubDate ? new Date(pubDate) : new Date(),
            };
        });
    } catch (err) {
        console.warn('[' + src.name + ']', err.message);
        return [];
    }
}

async function fetchAll() {
    const ldText = document.getElementById('loading-text');
    const ldBar  = document.getElementById('loading-bar');
    const ldSrc  = document.getElementById('loading-sources');
    const step   = 80 / SOURCES.length;
    let progress = 8;
    ldBar.style.width = progress + '%';

    const results = [];
    for (let i = 0; i < SOURCES.length; i++) {
        const src = SOURCES[i];
        ldText.innerHTML = 'FETCHING ' + src.name.toUpperCase() + '<span class="cursor">_</span>';
        ldSrc.textContent = 'Source ' + (i + 1) + ' of ' + SOURCES.length;
        const items = await fetchSource(src);
        results.push(...items);
        progress += step;
        ldBar.style.width = Math.min(progress, 92) + '%';
    }
    ldBar.style.width = '100%';
    return results;
}

/* ---- Cache ---- */

function saveCache(news) {
    try {
        localStorage.setItem('ainf_cache', JSON.stringify({
            ts: Date.now(), data: news.slice(0, 60)
        }));
    } catch (_) {}
}

function loadCache() {
    try {
        const raw = localStorage.getItem('ainf_cache');
        if (!raw) return null;
        const { ts, data } = JSON.parse(raw);
        if (Date.now() - ts > CFG.CACHE_TTL) return null;
        return data.map(n => ({ ...n, date: new Date(n.date) }));
    } catch (_) { return null; }
}

async function loadBackup() {
    try {
        const r = await fetch('backup.json');
        const d = await r.json();
        return d.map(n => ({ ...n, date: new Date(n.date) }));
    } catch (_) { return []; }
}

/* ---- Render ---- */

function renderFeed() {
    const feed     = document.getElementById('feed');
    const noRes    = document.getElementById('no-results');
    const storyEl  = document.getElementById('story-count');

    const items = currentFilter === 'all'
        ? allNews
        : allNews.filter(n => n.category.toLowerCase() === currentFilter);

    feed.innerHTML = '';
    if (items.length === 0) { noRes.style.display = 'block'; storyEl.textContent = '0 STORIES'; return; }
    noRes.style.display = 'none';
    storyEl.textContent = items.length + ' STORIES';

    const frag = document.createDocumentFragment();
    items.forEach((item, i) => frag.appendChild(createCard(item, i)));
    feed.appendChild(frag);
}

function bgForCat(cat) {
    return { AI: 'linear-gradient(135deg,#030316,#071a07,#030316)',
             LLM: 'linear-gradient(135deg,#0d0016,#150a2e,#0d0016)',
             RETAIL: 'linear-gradient(135deg,#00161a,#030316,#00161a)' }[cat]
           || 'linear-gradient(135deg,#030316,#0a0a1a)';
}

function createCard(item, index) {
    const card = document.createElement('article');
    card.className = 'news-card';
    card.style.setProperty('--scan-delay', (index * 0.6 % 4) + 's');

    const catCls = 'cat-' + item.category.toLowerCase();
    const catIco = CAT_ICONS[item.category] || '&#9674;';
    const ago    = timeAgo(item.date);
    const ph     = bgForCat(item.category);

    const imgPart = item.image
        ? `<img class="card-img" src="${esc(item.image)}" alt="" loading="lazy"
               onerror="this.style.display='none';this.nextElementSibling.style.display='flex'">`
        : '';
    const phStyle = item.image ? 'display:none' : '';

    card.innerHTML = `
        <div class="card-image-wrap">
            ${imgPart}
            <div class="card-img-placeholder" style="${phStyle};background:${ph}">
                <span class="p-icon ${catCls}">${catIco}</span>
            </div>
            <span class="badge badge-cat ${catCls}">${item.category}</span>
            <span class="badge badge-src">${esc(item.source)}</span>
        </div>
        <div class="card-content">
            <div class="card-headline">${esc(item.headline)}</div>
            <div class="card-summary">${esc(item.summary)}</div>
            <div class="card-meta">
                <span>${ago}</span>
                <span class="card-open">&#9658; OPEN</span>
            </div>
        </div>`;

    card.addEventListener('click', () => openViewer(item));
    return card;
}

/* ---- Ticker ---- */

function setTicker(news) {
    const ticker = document.getElementById('ticker');
    if (!news.length) return;
    const line = news.slice(0, 25)
        .map(n => CAT_ICONS[n.category] + ' ' + n.headline)
        .join('   &nbsp;&nbsp;&nbsp;&#9830;&nbsp;&nbsp;&nbsp;   ');
    ticker.innerHTML = '<span>' + line + '</span>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span>' + line + '</span>';
}

/* ---- Viewer ---- */

function openViewer(item) {
    currentItem = item;

    document.getElementById('viewer-title').textContent   = item.title;

    const catEl = document.getElementById('viewer-category');
    catEl.textContent  = item.category;
    catEl.className    = 'viewer-category badge cat-' + item.category.toLowerCase();

    const srcEl = document.getElementById('viewer-source');
    srcEl.textContent = item.source;
    srcEl.style.cssText = 'border-color:rgba(0,229,255,.4);color:var(--neon-cyan)';

    const dateEl = document.getElementById('viewer-date');
    dateEl.textContent = timeAgo(item.date);
    dateEl.style.cssText = 'color:var(--text-dim);border-color:var(--border-dim)';

    buildSummary(item);
    switchTab('summary');

    document.getElementById('viewer-overlay').classList.add('active');
    document.body.style.overflow = 'hidden';
}

function buildSummary(item) {
    const panel = document.getElementById('summary-content');
    const sentences = getSentences(item.fullText || item.summary, 6);
    const bullets = (sentences.length > 0 ? sentences : [item.summary])
        .map(s => `<li>${esc(s)}</li>`).join('');

    const imgHtml = item.image
        ? `<img class="summary-img" src="${esc(item.image)}" alt=""
               onerror="this.remove()">`
        : '';

    panel.innerHTML = `
        ${imgHtml}
        <ul class="summary-bullets">${bullets}</ul>
        <div class="summary-link-bar">
            <button class="open-source-btn" id="btn-goto-src">&#9674; VIEW SOURCE MATERIAL</button>
        </div>`;

    document.getElementById('btn-goto-src').addEventListener('click', () => switchTab('source'));
}

function switchTab(view) {
    document.querySelectorAll('.viewer-tab').forEach(t =>
        t.classList.toggle('active', t.dataset.view === view)
    );
    document.getElementById('panel-summary').style.display = view === 'summary' ? 'flex' : 'none';
    document.getElementById('panel-source').style.display  = view === 'source'  ? 'flex' : 'none';

    if (view === 'source' && currentItem) loadIframe(currentItem);
}

function loadIframe(item) {
    const iframe  = document.getElementById('viewer-iframe');
    const loading = document.getElementById('iframe-loading');
    const blocked = document.getElementById('iframe-blocked');

    iframe.style.display  = 'block';
    loading.style.display = 'flex';
    blocked.style.display = 'none';

    document.getElementById('blocked-link').href = item.link;

    clearTimeout(iframeTimer);

    iframe.onload = () => {
        loading.style.display = 'none';
        clearTimeout(iframeTimer);
        try {
            const body = iframe.contentDocument?.body;
            if (body && body.innerHTML.trim().length < 100) showBlocked();
        } catch (_) { /* cross-origin — content loaded fine */ }
    };
    iframe.onerror = () => { clearTimeout(iframeTimer); showBlocked(); };

    iframe.src = item.link;

    iframeTimer = setTimeout(() => {
        try {
            if (iframe.contentWindow?.location?.href === 'about:blank') showBlocked();
            else loading.style.display = 'none';
        } catch (_) { loading.style.display = 'none'; }
    }, CFG.IFRAME_TIMEOUT);
}

function showBlocked() {
    document.getElementById('iframe-loading').style.display   = 'none';
    document.getElementById('viewer-iframe').style.display    = 'none';
    document.getElementById('iframe-blocked').style.display   = 'flex';
}

function closeViewer() {
    document.getElementById('viewer-overlay').classList.remove('active');
    document.body.style.overflow = '';
    document.getElementById('viewer-iframe').src = 'about:blank';
    clearTimeout(iframeTimer);
    currentItem = null;
}

/* ---- Init ---- */

async function init(force) {
    const screen  = document.getElementById('loading-screen');
    const updated = document.getElementById('last-updated');

    /* Show cached data immediately */
    if (!force) {
        const cached = loadCache();
        if (cached?.length) {
            allNews = cached;
            renderFeed();
            setTicker(allNews);
            screen.style.display = 'none';
            updated.textContent  = 'FROM CACHE';
        }
    }

    if (!allNews.length || force) screen.style.display = 'flex';

    const fresh = await fetchAll();

    if (fresh.length) {
        allNews = fresh.sort((a, b) => new Date(b.date) - new Date(a.date));
        saveCache(allNews);
    } else if (!allNews.length) {
        const backup = await loadBackup();
        allNews = backup.length ? backup : [];
    }

    if (allNews.length) {
        renderFeed();
        setTicker(allNews);
        updated.textContent = 'UPDATED ' + new Date().toLocaleTimeString();
    } else {
        document.getElementById('loading-text').innerHTML =
            'FEED UNAVAILABLE — CHECK CONNECTION<span class="cursor">_</span>';
        return;
    }

    screen.style.display = 'none';

    clearInterval(refreshTimer);
    refreshTimer = setInterval(() => init(false), CFG.REFRESH_INTERVAL);
}

/* ---- Events ---- */

document.addEventListener('DOMContentLoaded', () => {
    /* Filter tabs */
    document.querySelectorAll('.tab').forEach(tab => {
        tab.addEventListener('click', () => {
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
            currentFilter = tab.dataset.filter;
            renderFeed();
        });
    });

    /* Viewer tabs */
    document.querySelectorAll('.viewer-tab').forEach(tab =>
        tab.addEventListener('click', () => switchTab(tab.dataset.view))
    );

    /* Close viewer */
    document.getElementById('viewer-close').addEventListener('click', closeViewer);
    document.getElementById('viewer-overlay').addEventListener('click', e => {
        if (e.target === e.currentTarget) closeViewer();
    });
    document.addEventListener('keydown', e => { if (e.key === 'Escape') closeViewer(); });

    /* Refresh */
    document.getElementById('refresh-btn').addEventListener('click', () => init(true));

    /* Start */
    init(false);
});
