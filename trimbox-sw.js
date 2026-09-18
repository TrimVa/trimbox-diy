// ============================================================================
//  TrimBox DIY — service worker
//  ---------------------------------------------------------------------------
//  Optionnel : la console fonctionne intégralement sans ce fichier (elle est
//  déjà autonome, sans dépendance externe). Ce qu'il ajoute :
//   1. L'invite d'installation automatique de Chrome (bannière / mini-barre),
//      qui exige un gestionnaire fetch() réellement fonctionnel — un
//      gestionnaire vide est désormais ignoré par Chrome.
//   2. Un chargement instantané même hors ligne dès la deuxième visite,
//      la page étant servie depuis le cache plutôt que depuis le réseau.
//
//  À déposer À CÔTÉ de trimbox-diy-console.html (même dossier, même origine) :
//  un service worker ne peut pas être chargé depuis un autre domaine, ni
//  embarqué en ligne dans le HTML (contrainte de la plateforme web, pas un
//  choix de conception ici).
// ============================================================================

const CACHE_NAME = 'trimbox-diy-v1';

// Seule la page elle-même est mise en cache : la console n'a aucune autre
// ressource (pas de feuille de style, pas de script, pas d'image externes).
const SHELL = [
  './trimbox-diy-console.html',
];

self.addEventListener('install', (event) => {
  self.skipWaiting();
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => cache.addAll(SHELL).catch(() => {}))
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((names) =>
      Promise.all(names.filter((n) => n !== CACHE_NAME).map((n) => caches.delete(n)))
    ).then(() => self.clients.claim())
  );
});

// Stratégie « cache d'abord, réseau en secours » : la page s'ouvre
// instantanément depuis le cache, et se met à jour en arrière-plan dès
// qu'une connexion est disponible.
self.addEventListener('fetch', (event) => {
  if (event.request.method !== 'GET') return;

  event.respondWith(
    caches.match(event.request).then((cached) => {
      const network = fetch(event.request)
        .then((response) => {
          if (response && response.ok) {
            const copy = response.clone();
            caches.open(CACHE_NAME).then((cache) => cache.put(event.request, copy));
          }
          return response;
        })
        .catch(() => cached);
      return cached || network;
    })
  );
});
