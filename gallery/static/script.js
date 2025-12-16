let currentDate = new Date();
let photos = [];

document.addEventListener('DOMContentLoaded', async () => {
    await fetchPhotos();
    renderCalendar();
});

async function fetchPhotos() {
    try {
        const response = await fetch('/api/photos');
        photos = await response.json();
        console.log("Loaded photos:", photos);
    } catch (error) {
        console.error("Error loading photos:", error);
    }
}

function renderCalendar() {
    const monthYear = document.getElementById('month-year');
    const daysGrid = document.getElementById('days-grid');

    // Set Month Year Title
    const options = { month: 'long', year: 'numeric' };
    monthYear.textContent = currentDate.toLocaleDateString('en-US', options);

    daysGrid.innerHTML = '';

    const year = currentDate.getFullYear();
    const month = currentDate.getMonth(); // 0-indexed

    // First day of month
    const firstDay = new Date(year, month, 1);
    const lastDay = new Date(year, month + 1, 0);

    // 0 = Sunday, 1 = Monday ...
    const startDayIndex = firstDay.getDay();
    const totalDays = lastDay.getDate();

    // Previous month empty cells
    for (let i = 0; i < startDayIndex; i++) {
        const empty = document.createElement('div');
        empty.classList.add('day-cell');
        empty.style.border = 'none'; // Invisible
        daysGrid.appendChild(empty);
    }

    // Days
    for (let d = 1; d <= totalDays; d++) {
        const cell = document.createElement('div');
        cell.classList.add('day-cell');

        // Check day of week for styling
        const currentDayOfWeek = new Date(year, month, d).getDay();
        if (currentDayOfWeek === 0) cell.classList.add('sunday-cell');
        if (currentDayOfWeek === 6) cell.classList.add('saturday-cell');

        // Date Number
        const num = document.createElement('div');
        num.classList.add('day-number');
        num.textContent = d;
        cell.appendChild(num);

        // Find Photo for this date
        const dateString = `${year}-${String(month + 1).padStart(2, '0')}-${String(d).padStart(2, '0')}`;
        const photo = photos.find(p => p.date === dateString);

        if (photo) {
            const img = document.createElement('img');
            img.src = `/captures/${photo.filename}`;
            img.classList.add('photo-thumb');
            img.onclick = () => openModal(img.src, dateString);
            cell.appendChild(img);
        }

        daysGrid.appendChild(cell);
    }
}

function prevMonth() {
    currentDate.setMonth(currentDate.getMonth() - 1);
    renderCalendar();
}

function nextMonth() {
    currentDate.setMonth(currentDate.getMonth() + 1);
    renderCalendar();
}

// Modal
function openModal(src, caption) {
    const modal = document.getElementById("photo-modal");
    const modalImg = document.getElementById("modal-img");
    const captionText = document.getElementById("caption");

    modal.style.display = "block";
    modalImg.src = src;
    captionText.innerHTML = caption;
}

function closeModal() {
    document.getElementById("photo-modal").style.display = "none";
}
